#!/usr/bin/env python3
import collections
import time

import cv_bridge
import message_filters
import numpy as np
import rclpy
import torch
from franky_msgs.msg import CartesianMove, GripperGrasp, GripperState
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from il_recorder.pointcloud_utils import flat_pc_from_ros, idp3_preprocess_point_cloud
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

# policy code
from .HumanScoredFlowMatching.flow_policy.train import TrainDP3Workspace

# --- CONFIGURATION ---
HISTORY_LENGTH = 2  # How many past steps to condition on (RTC)
EXECUTION_HORIZON = 8  # How many steps we buffer/execute
CONTROL_RATE = 15.0  # Hz


def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class FlowInferenceNode(Node):
    def __init__(self):
        super().__init__("flow_inference_node")

        # Load model
        ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/epoch=0300-test_mean_score=-0.015.ckpt"
        self.get_logger().info(f"Loading checkpoint: {ckpt_path}")

        self.workspace = TrainDP3Workspace.create_from_checkpoint(ckpt_path)
        self.workspace.model.eval()
        self.workspace.model.cuda()

        # RTC Buffers
        self.obs_window_size = 2
        self.states_deque = collections.deque(maxlen=self.obs_window_size)
        self.pcd_deque = collections.deque(maxlen=self.obs_window_size)

        # Action Queues
        self.action_buffer = []
        self.action_history = collections.deque(maxlen=HISTORY_LENGTH)

        # Gripper status
        self.last_gripper_cmd = 0
        self.last_pos = None
        self.is_shutting_down = False

        # ROS
        cb_group = ReentrantCallbackGroup()

        self.pose_callback = self.create_subscription(
            PoseStamped,
            "/fr3/end_effector_pose",
            self.feedback_callback,
            10,
            callback_group=cb_group,
        )

        self.sub_pts = message_filters.Subscriber(
            self, PointCloud2, "/camera1/depth/color/points"
        )
        self.sub_joints = message_filters.Subscriber(
            self, JointState, "/fr3/joint_states"
        )
        self.sub_gripper = message_filters.Subscriber(
            self, GripperState, "/fr3/gripper_state"
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_joints, self.sub_pts, self.sub_gripper],
            queue_size=2,
            slop=0.1,
            allow_headerless=True,
        )
        self.ts.registerCallback(self.infer_callback)

        # Robot Command
        self.pub_pose = self.create_publisher(
            CartesianMove, "/fr3/cartesian_pose_cmd", 10
        )
        self.pub_gripper = self.create_publisher(GripperGrasp, "/fr3/gripper_grasp", 10)

        # --- 4. CONTROL LOOP TIMER ---
        self.timer = self.create_timer(
            1.0 / CONTROL_RATE, self.control_loop, callback_group=cb_group
        )

        self.get_logger().info("Flow Inference Node Ready.")

    def feedback_callback(self, msg: PoseStamped):
        """Updates the robot's current pose."""

        xyz = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ]
        )
        rpy = R.from_quat(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        ).as_euler("xyz")

        self.last_pos = np.concatenate([xyz, rpy])

    def infer_callback(self, joint_msg, pc_msg, gripper_msg):
        # If we have no history, we wait for our current pose to fill with.
        if len(self.action_history) == 0:
            if self.last_pos is not None:
                self.get_logger().info("Seeding History with Current Pose...")

                for _ in range(HISTORY_LENGTH):
                    pos = self.last_pos.copy()
                    gripper = 0.0 if gripper_msg.width > 0.01 else 1.0  # open vs closed
                    self.action_history.append(np.concatenate([pos, [gripper]]))
            else:
                return  # Wait for feedback callback

        # Skip inference if buffer is full
        if len(self.action_buffer) > 2:
            return

        # Get and process point cloud
        pcloud = flat_pc_from_ros(pc_msg, remove_nans=True)
        pcloud = idp3_preprocess_point_cloud(
            pcloud, num_points=4096, near=0.1, far=1.5
        )  # TODO confirm near, far

        # gripper_pos = 0.0 if joint_msg.position[7] < 0.1 else 1.0
        # joints = np.concatenate([joint_msg.position[:7], [gripper_pos]])
        joints = np.array(joint_msg.position[:7], dtype=np.float32)

        # NOTE these have maxlen so they will automatically pop old entries
        self.states_deque.append(joints)
        self.pcd_deque.append(pcloud)

        if len(self.states_deque) < self.obs_window_size:
            return

        # Prepare Tensors
        state_input = np.array(self.states_deque)
        pc_input = np.array(self.pcd_deque)

        data = {
            "obs": {
                "agent_pos": torch.from_numpy(state_input).unsqueeze(0).cuda().float(),
                "point_cloud": torch.from_numpy(pc_input).unsqueeze(0).cuda().float(),
            }
        }

        # 6. Condition on History (RTC)
        past_actions_arr = np.array(self.action_history)
        past_actions_tensor = (
            torch.from_numpy(past_actions_arr).unsqueeze(0).cuda().float()
        )

        # 7. Inference
        with torch.no_grad():
            result = self.workspace.model.predict_action(
                data["obs"], past_actions=past_actions_tensor
            )
            raw_action_seq = result["action"].squeeze(0).cpu().numpy()

        # 8. RTC Slicing
        # Skip the first HISTORY_LENGTH steps (reconstructed past)
        # Keep the future steps
        future_actions = raw_action_seq[HISTORY_LENGTH:]

        self.action_buffer = []
        gripper_actions = raw_action_seq[HISTORY_LENGTH:, 6]
        consensus = 1 if np.mean(gripper_actions) > 0.5 else 0

        future_actions[:, 6] = consensus
        self.action_buffer.extend(future_actions[:EXECUTION_HORIZON])

    def control_loop(self):
        if not self.action_buffer:
            return

        # 1. Pop Action
        target_pose = self.action_buffer.pop(0)

        # 2. Update History (for future conditioning)
        # NOTE again a deque with maxlen so we keep only latest HISTORY_LENGTH entries
        self.action_history.append(target_pose)

        gripper = target_pose[6]
        if gripper != self.last_gripper_cmd:
            # send gripper target
            self.last_gripper_cmd = gripper

            # 0 if command is 1 (close), else 0.08 (open)
            # TODO have to confirm this...
            grip_width = (1 - gripper) * 0.08
            gripper_msg = GripperGrasp()
            gripper_msg.width = float(grip_width)
            gripper_msg.speed = 0.1
            gripper_msg.force = 0.01
            gripper_msg.epsilon_inner = 0.3
            gripper_msg.epsilon_outer = 0.3

            if not self.is_shutting_down:
                self.pub_gripper.publish(gripper_msg)

        target_msg = CartesianMove()
        target_msg.pose.position.x = float(target_pose[0])
        target_msg.pose.position.y = float(target_pose[1])
        target_msg.pose.position.z = float(target_pose[2])
        rpy = target_pose[3:6]
        rpy = wrap_to_pi(rpy)
        quat = R.from_euler("xyz", rpy).as_quat()
        target_msg.pose.orientation.x = quat[0]
        target_msg.pose.orientation.y = quat[1]
        target_msg.pose.orientation.z = quat[2]
        target_msg.pose.orientation.w = quat[3]

        target_msg.relative = False  # absolute position control

        self.get_logger().info(
            "Current Pose: " + ", ".join(f"{x:.3f}" for x in self.last_pos)
        )
        self.get_logger().info(
            "Target Pose: " + ", ".join(f"{x:.3f}" for x in target_pose[:6])
        )

        if not self.is_shutting_down:
            self.pub_pose.publish(target_msg)

    def stop_robot(self):
        # Publish zero velocity command to stop the robot
        self.is_shutting_down = True
        stop_msg = CartesianMove()
        stop_msg.pose.position.x = self.last_pos[0]
        stop_msg.pose.position.y = self.last_pos[1]
        stop_msg.pose.position.z = self.last_pos[2]
        rpy = self.last_pos[3:6]
        rpy = wrap_to_pi(rpy)
        quat = R.from_euler("xyz", rpy).as_quat()
        stop_msg.pose.orientation.x = quat[0]
        stop_msg.pose.orientation.y = quat[1]
        stop_msg.pose.orientation.z = quat[2]
        stop_msg.pose.orientation.w = quat[3]

        stop_msg.relative = False  # absolute position control

        self.get_logger().info("Sending current pose to robot...")
        self.pub_pose.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = FlowInferenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Stopping...")
    finally:
        node.stop_robot()
        time.sleep(1)  # Give some time for the stop command to be sent
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
