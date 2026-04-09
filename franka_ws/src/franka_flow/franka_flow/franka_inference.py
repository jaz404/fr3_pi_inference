#!/usr/bin/env python3
import collections
import time

import cv2
import cv_bridge
import message_filters
import numpy as np
import rclpy
import torch
from franky_msgs.msg import (
    CartesianMove,
    CorrectionInfo,
    GripperGrasp,
    GripperMove,
    GripperState,
)
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from il_recorder.pointcloud_utils import flat_pc_from_ros, idp3_preprocess_point_cloud
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, JointState, Joy, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool

# policy code
from .HumanScoredFlowMatching.flow_policy.train import TrainDP3Workspace

# --- CONFIGURATION ---
HISTORY_LENGTH = 2  # How many past steps to condition on (RTC)
EXECUTION_HORIZON = 3  # How many steps we buffer/execute
CONTROL_RATE = 5.0  # Hz

MULTIPLE_TAKEOVER = "multiple"
SINGLE_TAKEOVER = "single"


def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class FlowInferenceNode(Node):
    def __init__(self):
        super().__init__("flow_inference_node")

        # Load model
        # ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/epoch=0300-test_mean_score=-0.015.ckpt"
        # ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/peartabletest_epoch=0300-test_mean_score=-0.025.ckpt"
        # ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/franka_peartable_test-epoch=0180-test_mean_score=-0.037.ckpt"
        # ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/checkpoints/franka_peartable_test60-epoch=0150-test_mean_score=-0.032.ckpt"
        # ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/checkpoints/franka_peartable_test60-epoch=0120-test_mean_score=-0.038.ckpt"
        # ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/checkpoints/franka_peartable_test60-epoch=0180-test_mean_score=-0.030.ckpt"
        # ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/checkpoints_cart/franka_peartable_test60-epoch=0180-test_mean_score=-0.031.ckpt"

        # ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/siriusrgb/franka_banana_rgb-epoch=0160-test_mean_score=-0.037.ckpt"
        # ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/imgpolicy/franka_banana_rgb_30-epoch=0160-test_mean_score=-0.044.ckpt"
        # ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/imgpolicy/franka_banana_rgb_40-epoch=0160-test_mean_score=-0.043.ckpt"
        # ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/imgpolicy/franka_banana_rgb_50-epoch=0160-test_mean_score=-0.039.ckpt"
        # ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/USER_STUDY/cole/franka_banana_rgb_new-epoch=0160-test_mean_score=-0.035.ckpt"
        # ckpt_path = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/USER_STUDY/base.ckpt"

        # shorthands for eval
        USER = "tanner"
        # ckpt_path = f"/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/USER_STUDY/{USER}/single/round1.ckpt"
        # ckpt_path = f"/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/USER_STUDY/{USER}/single/round2.ckpt"
        ckpt_path = f"/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/USER_STUDY/{USER}/multiple/round1.ckpt"
        # ckpt_path = f"/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/USER_STUDY/{USER}/multiple/round2.ckpt"

        self.declare_parameter("ckpt_path", "")
        ckpt_path_arg = (
            self.get_parameter("ckpt_path").get_parameter_value().string_value
        )
        ckpt_path = ckpt_path_arg if ckpt_path_arg else ckpt_path

        self.get_logger().info(f"Loading checkpoint: {ckpt_path}")

        self.declare_parameter(
            "takeover_type", MULTIPLE_TAKEOVER
        )  # either single or multiple
        self.takeover_type = (
            self.get_parameter("takeover_type").get_parameter_value().string_value
        )
        self.declare_parameter("doing_corrections", False)
        self.doing_corrections = (
            self.get_parameter("doing_corrections").get_parameter_value().bool_value
        )
        self.declare_parameter("conditioning_type", "cartesian")  # cartesian, joint
        self.conditioning_type = (
            self.get_parameter("conditioning_type").get_parameter_value().string_value
        )
        self.declare_parameter("obs_type", "pointcloud")
        self.obs_type = (
            self.get_parameter("obs_type").get_parameter_value().string_value
        )

        self.user_takeover = False
        self.is_recording = False

        self.workspace = TrainDP3Workspace.create_from_checkpoint(ckpt_path)
        self.workspace.model.eval()
        self.workspace.model.cuda()

        # RTC Buffers
        self.obs_window_size = 2
        self.jnt_states_deque = collections.deque(maxlen=self.obs_window_size)
        self.gripper_deque = collections.deque(maxlen=self.obs_window_size)
        self.cart_states_deque = collections.deque(maxlen=self.obs_window_size)
        self.pcd_deque = collections.deque(maxlen=self.obs_window_size)
        self.img_deque = collections.deque(maxlen=self.obs_window_size)

        # Action Queues
        self.action_buffer = []
        self.action_history = collections.deque(maxlen=HISTORY_LENGTH)

        # Gripper status
        self.last_gripper_cmd = 0
        self.last_gripper_time = time.time()
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
        self.sub_img = message_filters.Subscriber(
            self, Image, "/camera1/color/image_raw"
        )
        self.sub_joints = message_filters.Subscriber(
            self, JointState, "/fr3/joint_states"
        )
        self.sub_gripper = message_filters.Subscriber(
            self, GripperState, "/fr3/gripper_state"
        )

        if self.doing_corrections:
            self.sub_joy = message_filters.Subscriber(self, Joy, "joy")
            self.sub_is_recording = self.create_subscription(
                Bool,
                "is_recording",
                self.is_recording_callback,
                10,
                callback_group=cb_group,
            )

        connections = [self.sub_joints, self.sub_gripper, self.sub_pts, self.sub_img]

        if self.doing_corrections:
            connections.append(self.sub_joy)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            connections,
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
        self.pub_gripper_move = self.create_publisher(
            GripperMove, "/fr3/gripper_move", 10
        )
        self.pub_correction_info = self.create_publisher(
            CorrectionInfo, "correction_info", 10
        )

        # --- 4. CONTROL LOOP TIMER ---
        self.timer = self.create_timer(
            1.0 / CONTROL_RATE, self.control_loop, callback_group=cb_group
        )

        self.get_logger().info("Flow Inference Node Ready.")

    def is_recording_callback(self, msg: Bool):
        """Resets buffers when a new recording starts."""
        self.is_recording = msg.data

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

    def infer_callback(self, joint_msg, gripper_msg, pc_msg, img_msg, joy_msg=None):
        # If we have no history, we wait for our current pose to fill with.
        if len(self.action_history) == 0:
            if self.last_pos is not None:
                self.get_logger().info("Seeding History with Current Pose...")

                for _ in range(HISTORY_LENGTH):
                    pos = self.last_pos.copy()
                    gripper = 0.0 if gripper_msg.width > 0.07 else 1.0  # open vs closed
                    self.action_history.append(np.concatenate([pos, [gripper]]))
            else:
                return  # Wait for feedback callback

        gripper = 0.0 if gripper_msg.width > 0.07 else 1.0  # open vs closed

        if joy_msg and self.doing_corrections:
            # triggers = np.array(joy_msg.axes)[[2, 5]] # xbox
            triggers = np.array(joy_msg.axes)[[4, 5]]  # ps4
            # axes = np.array(joy_msg.axes)[[0, 1, 3, 4, 6, 7]] # xbox
            axes = np.array(joy_msg.axes)[[0, 1, 2, 3]]  # ps4
            on_toggle = joy_msg.buttons[0] == 1.0  # A button for xbox, X button for ps4
            off_toggle = (
                joy_msg.buttons[1] == 1.0
            )  # B button for xbox, Circle button for ps4
            episode_toggle = joy_msg.buttons[6] == 1.0  # Options ps4

            # self.get_logger().info(f"On Toggle: {on_toggle}, Off Toggle: {off_toggle}")

            if on_toggle:
                self.user_takeover = True
                self.get_logger().info("User Takeover Activated")
                return
            elif episode_toggle:
                # So that we can still give control back when doing single takeover, we reset here at start/end of episode.
                self.user_takeover = False
                self.action_buffer = []
                self.action_history = collections.deque(maxlen=HISTORY_LENGTH)
                self.jnt_states_deque.clear()
                self.gripper_deque.clear()
                self.cart_states_deque.clear()
                self.pcd_deque.clear()
                return
            elif (
                self.user_takeover
                and self.takeover_type == MULTIPLE_TAKEOVER
                and off_toggle
            ):
                self.get_logger().info("User Takeover Stopped")

                self.user_takeover = False
                self.action_buffer = []
                self.action_history = collections.deque(maxlen=HISTORY_LENGTH)
                self.jnt_states_deque.clear()
                self.gripper_deque.clear()
                self.cart_states_deque.clear()
                self.pcd_deque.clear()
                return

        if self.doing_corrections:
            self.pub_correction_info.publish(
                CorrectionInfo(human_takeover=self.user_takeover)
            )

        # wait until we start demo so we synch up with il recorder code
        if self.doing_corrections and not self.is_recording:
            return

        # Skip inference if buffer is full
        if len(self.action_buffer) > 2:
            return

        # NOTE: might switch to button switch control but you will also have to send topic to joy to stop that control
        # if self.doing_corrections and (
        #     np.any(np.abs(axes) > 0.1) or np.any(triggers < 0.9)
        # ):
        #     self.get_logger().info(
        #         f"Manual control input detected, skipping inference... {axes} {triggers}"
        #     )
        #     self.user_takeover = True
        #     return
        # elif (
        #     self.doing_corrections
        #     and self.user_takeover
        #     and self.takeover_type == MULTIPLE_TAKEOVER
        # ):
        #     self.user_takeover = False
        #     self.action_buffer = []
        #     self.action_history = collections.deque(maxlen=HISTORY_LENGTH)
        #     self.jnt_states_deque.clear()
        #     self.gripper_deque.clear()
        #     self.cart_states_deque.clear()
        #     self.pcd_deque.clear()
        #     return

        # Get and process point cloud
        pcloud = flat_pc_from_ros(pc_msg, remove_nans=True)
        pcloud = idp3_preprocess_point_cloud(
            pcloud, num_points=4096, near=0.1, far=1.5
        )  # TODO confirm near, far

        # Process images
        img_1d = np.frombuffer(img_msg.data, dtype=np.uint8).copy()
        cv_img = img_1d.reshape((img_msg.height, img_msg.width, 3))
        if "rgb" in img_msg.encoding.lower():
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        cv_img = cv2.resize(cv_img, (84, 84), interpolation=cv2.INTER_AREA)

        gripper = 0.0 if gripper_msg.width > 0.07 else 1.0  # open vs closed
        # joints = np.concatenate([joint_msg.position[:7], [gripper_pos]])
        joints = np.array(joint_msg.position[:7], dtype=np.float32)

        # NOTE these have maxlen so they will automatically pop old entries
        self.jnt_states_deque.append(joints)
        self.gripper_deque.append(gripper)
        self.cart_states_deque.append(self.last_pos)
        self.pcd_deque.append(pcloud)
        self.img_deque.append(cv_img)

        if len(self.jnt_states_deque) < self.obs_window_size:
            return
        if len(self.cart_states_deque) < self.obs_window_size:
            return

        # Prepare Tensors
        jnt_state_input = np.array(self.jnt_states_deque)
        gripper_input = np.array(self.gripper_deque)[:, np.newaxis]
        cart_state_input = np.array(self.cart_states_deque)
        gripper_input = np.array(self.gripper_deque)[:, np.newaxis]
        # self.get_logger().info(f"{cart_state_input} {gripper_input}")
        state_input = (
            np.concatenate([cart_state_input, gripper_input], axis=1)
            if self.conditioning_type == "cartesian"
            else np.concatenate([jnt_state_input, gripper_input], axis=1)
        )
        pc_input = np.array(self.pcd_deque)
        img_input = np.array(self.img_deque).transpose(0, 3, 1, 2)

        data = {
            "obs": {
                "agent_pos": torch.from_numpy(state_input).unsqueeze(0).cuda().float(),
                # "point_cloud": torch.from_numpy(pc_input).unsqueeze(0).cuda().float(),
            }
        }
        if self.obs_type == "pointcloud":
            data["obs"]["point_cloud"] = (
                torch.from_numpy(pc_input).unsqueeze(0).cuda().float()
            )
        else:
            data["obs"]["img"] = torch.from_numpy(img_input).unsqueeze(0).cuda().float()

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

        self.get_logger().info(f"{np.mean(gripper_actions)}, {consensus=}")

        future_actions[:, 6] = consensus
        self.action_buffer.extend(future_actions[:EXECUTION_HORIZON])

    def control_loop(self):
        if not self.action_buffer or self.user_takeover:
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
            self.last_gripper_time = time.time()

            self.get_logger().info(
                f"Gripper Command: {'Close' if gripper == 1.0 else 'Open'}"
            )

            if gripper == 1.0 and not self.is_shutting_down:
                gripper_msg = GripperGrasp()
                gripper_msg.width = 0.0
                gripper_msg.speed = 0.1
                gripper_msg.force = 10.0
                gripper_msg.epsilon_inner = 0.8
                gripper_msg.epsilon_outer = 0.8
                self.pub_gripper.publish(gripper_msg)
            elif not self.is_shutting_down:
                msg = GripperMove()
                msg.width = 0.08
                msg.speed = 0.1
                self.pub_gripper_move.publish(msg)

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

        # self.get_logger().info(
        #     "Current Pose: " + ", ".join(f"{x:.3f}" for x in self.last_pos)
        # )
        # self.get_logger().info(
        #     "Target Pose: " + ", ".join(f"{x:.3f}" for x in target_pose[:6])
        # )

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
