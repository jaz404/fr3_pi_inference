from time import sleep

import franky
import numpy as np
import panda_py
import rclpy
import tf2_ros
from franky_msgs.msg import (
    CartesianMove,
    CartesianVelocity,
    GripperGrasp,
    GripperMove,
    GripperState,
    JointMove,
    JointVelocity,
)
from franky_msgs.srv import (
    BlockingCartesianMove,
    BlockingGripperGrasp,
    BlockingGripperMove,
    BlockingJointMove,
    SetDynamicsFactor,
)
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty


class FrankyRosBridge(Node):
    def __init__(self):
        super().__init__("franky_ros_bridge")

        self.declare_parameter("robot_ip", "192.168.1.105")
        self.declare_parameter("realtime", False)
        self.declare_parameter("max_velocity", 0.05)
        self.declare_parameter("max_acceleration", 0.1)
        self.declare_parameter("max_jerk", 0.15)
        robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        realtime = self.get_parameter("realtime").get_parameter_value().bool_value
        max_vel = self.get_parameter("max_velocity").get_parameter_value().double_value
        max_acc = (
            self.get_parameter("max_acceleration").get_parameter_value().double_value
        )
        max_jerk = self.get_parameter("max_jerk").get_parameter_value().double_value

        self.get_logger().info(f"Connecting to fr3 and Gripper at {robot_ip}...")

        self.desk = panda_py.Desk(robot_ip, "jagersand", "jagersand", platform="fr3")
        self.desk.take_control(force=True)
        self.get_logger().info(str(self.desk.has_control()))
        self.desk.unlock()
        self.desk.activate_fci()

        self.get_logger().info("Establishing hardware connection...")
        try:
            rt_conf = (
                franky.RealtimeConfig.Enforce
                if realtime
                else franky.RealtimeConfig.Ignore
            )
            self.robot = franky.Robot(robot_ip, realtime_config=rt_conf)
            # BUG: this part does not work as expected but keeping it for now due to nice ros params
            # self.robot.relative_dynamics_factor = franky.RelativeDynamicsFactor(
            #     velocity=max_vel, acceleration=max_acc, jerk=max_jerk
            # )
            self.robot.relative_dynamics_factor = 0.05
            self.get_logger().info(
                f"DYNAMICS: vel={self.robot.relative_dynamics_factor.velocity}, acc={self.robot.relative_dynamics_factor.acceleration}, jerk={self.robot.relative_dynamics_factor.jerk}"
            )
            self.gripper = franky.Gripper(robot_ip)
            self.get_logger().info("Hardware connection established.")
            self.get_logger().info(f"Gripper max width: {self.gripper.max_width}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to robot: {str(e)}")
            raise e

        # 3. Callback Groups
        self.publisher_group = MutuallyExclusiveCallbackGroup()
        self.arm_group = MutuallyExclusiveCallbackGroup()
        self.gripper_group = MutuallyExclusiveCallbackGroup()

        # 4. Publishers
        # Still using JointState for the state publisher as it's required for robot_state_publisher
        self.joint_pub = self.create_publisher(JointState, "fr3/joint_states", 10)
        self.ee_pose_pub = self.create_publisher(
            PoseStamped, "fr3/end_effector_pose", 10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.gripper_state_publisher = self.create_publisher(
            GripperState, "fr3/gripper_state", 10
        )

        # 5. Timer (State Update at 30Hz)
        self.timer = self.create_timer(
            1.0 / 30.0, self.publish_state_callback, callback_group=self.publisher_group
        )

        # 6. Subscribers (Arm Control)
        self.sub_joint_pos = self.create_subscription(
            JointMove,
            "fr3/joint_pos_cmd",
            self.joint_pos_callback,
            10,
            callback_group=self.arm_group,
        )

        self.sub_joint_vel = self.create_subscription(
            JointVelocity,
            "fr3/joint_vel_cmd",
            self.joint_vel_callback,
            10,
            callback_group=self.arm_group,
        )

        self.sub_cart_pose = self.create_subscription(
            CartesianMove,
            "fr3/cartesian_pose_cmd",
            self.cart_pose_callback,
            10,
            callback_group=self.arm_group,
        )
        self.sub_cart_vel = self.create_subscription(
            CartesianVelocity,
            "fr3/cartesian_vel_cmd",
            self.cart_vel_callback,
            10,
            callback_group=self.arm_group,
        )
        self.join_motion_callback = self.create_subscription(
            Empty,
            "fr3/join_motion",
            self.join_motion_callback,
            10,
            callback_group=self.arm_group,
        )

        # 7. Subscribers (Gripper Control)
        self.sub_grip_move = self.create_subscription(
            GripperMove,
            "fr3/gripper_move",
            self.gripper_move_callback,
            10,
            callback_group=self.gripper_group,
        )
        self.sub_grip_grasp = self.create_subscription(
            GripperGrasp,
            "fr3/gripper_grasp",
            self.gripper_grasp_callback,
            10,
            callback_group=self.gripper_group,
        )

        # Services
        self.srv_set_dynamics = self.create_service(
            SetDynamicsFactor,
            "fr3/set_dynamics_factor",
            self.set_dynamics_factor_callback,
            callback_group=self.arm_group,
        )
        self.srv_joint_move = self.create_service(
            BlockingJointMove,
            "fr3/move_joints_sync",
            self.move_joints_sync_callback,
            callback_group=self.arm_group,
        )
        self.srv_cart_move = self.create_service(
            BlockingCartesianMove,
            "fr3/move_cartesian_sync",
            self.move_cartesian_sync_callback,
            callback_group=self.arm_group,
        )

        self.srv_gripper_move = self.create_service(
            BlockingGripperMove,
            "fr3/move_gripper_sync",
            self.move_gripper_sync_callback,
            callback_group=self.gripper_group,
        )
        self.srv_gripper_grasp = self.create_service(
            BlockingGripperGrasp,
            "fr3/grasp_gripper_sync",
            self.grasp_gripper_sync_callback,
            callback_group=self.gripper_group,
        )

        self.joint_names = [f"fr3_joint{i+1}" for i in range(7)]
        self.get_logger().info("Franky ROS 2 Bridge is ready.")

    def publish_state_callback(self):
        """Fetches robot state and updates Publishers + TF tree."""
        state = self.robot.state
        now = self.get_clock().now().to_msg()

        # --- A. Publish JointState ---
        js_msg = JointState()
        js_msg.header.stamp = now
        js_msg.name = self.joint_names
        js_msg.position = list(state.q)
        js_msg.velocity = list(state.dq)
        js_msg.effort = list(state.tau_J)
        self.joint_pub.publish(js_msg)

        # Gripper State
        grip_state = self.gripper.state
        grip_msg = GripperState()
        grip_msg.width = grip_state.width
        grip_msg.is_grasped = grip_state.is_grasped
        self.gripper_state_publisher.publish(grip_msg)

        # --- B. Compute Cartesian Pose ---
        m = np.array(state.O_T_EE.matrix).reshape(4, 4, order="F")
        pos = m[:3, 3]
        rot_mat = m[:3, :3]
        quat = Rotation.from_matrix(rot_mat).as_quat()

        # --- C. Publish PoseStamped ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "fr3_link0"
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = (
            pos
        )
        (
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ) = quat
        self.ee_pose_pub.publish(pose_msg)

        # --- D. Broadcast End Effector TF Frame ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "fr3_link0"
        t.child_frame_id = "fr3_EE"
        (
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ) = pos
        (
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ) = quat
        self.tf_broadcaster.sendTransform(t)

    def set_dynamics_factor_callback(
        self, request: SetDynamicsFactor.Request, response: SetDynamicsFactor.Response
    ):
        """Sets the robot's relative dynamics factor based on the service request."""
        try:
            self.robot.relative_dynamics_factor = franky.RelativeDynamicsFactor(
                velocity=request.velocity_limit,
                acceleration=request.acceleration_limit,
                jerk=request.jerk_limit,
            )
            self.get_logger().info(
                f"Set dynamics factor - Vel: {request.velocity_limit}, Acc: {request.acceleration_limit}, Jerk: {request.jerk_limit}"
            )
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to set dynamics factor: {str(e)}")
            response.success = False
        return response

    # --- Arm Control Callbacks ---
    def join_motion_callback(self, msg: Empty):
        """Joins all async motions to the main thread. Can be used to try to clear any errors and catch exceptions."""
        try:
            self.robot.join_motion()
        except Exception as e:
            self.get_logger().error(f"Exception raised on joining motion: {str(e)}")
            self.robot.recover_from_errors()

    def joint_pos_callback(self, msg: JointMove):
        """Moves to target joint positions using a simple list of floats."""
        self.get_logger().info("Joint Position Command Received.")
        movement = franky.JointMotion(
            target=list(msg.positions),
            reference_type=(
                franky.ReferenceType.Relative
                if msg.relative
                else franky.ReferenceType.Absolute
            ),
        )
        try:
            self.robot.move(movement, asynchronous=True)
        except Exception as e:
            self.get_logger().error(f"joint position move failed: {str(e)}")
            self.robot.recover_from_errors()

    def joint_vel_callback(self, msg: JointVelocity):
        """Executes joint velocity motion using a simple list of floats."""
        movement = franky.JointVelocityMotion(
            target=list(msg.vels), duration=franky.Duration(msg.duration_millis)
        )
        try:
            self.robot.move(movement, asynchronous=True)
        except Exception as e:
            self.get_logger().error(f"joint velocity move failed: {str(e)}")
            self.robot.recover_from_errors()

    def cart_pose_callback(self, msg: CartesianMove):

        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]

        # NOTE COLE here we want relative to BASE frame so we have to hack this a bit
        if msg.relative:
            pose = self.robot.current_pose.end_effector_pose

            rel_pos = franky.Affine(pos)
            rel_ori = franky.Affine([0.0, 0.0, 0.0], quat)

            target = rel_pos * pose * rel_ori
            # self.get_logger().info(
            #     f"{rel_target.translation=}, {pose.translation=}, {target.translation=}"
            # )
        else:
            target = franky.Affine(pos, quat)

        movement = franky.CartesianMotion(
            target,
            # reference_type=(
            #     franky.ReferenceType.Relative
            #     if msg.relative
            #     else franky.ReferenceType.Absolute
            # ),
            reference_type=franky.ReferenceType.Absolute,
        )
        try:
            self.robot.move(movement, asynchronous=True)
        except Exception as e:
            self.robot.recover_from_errors()
            self.get_logger().error(f"cartesian pose move failed: {str(e)}")

    def cart_vel_callback(self, msg: CartesianVelocity):
        movement = franky.CartesianVelocityMotion(
            target=franky.Twist(list(msg.twist)[:3], list(msg.twist)[3:]),
            duration=franky.Duration(msg.duration_millis),
        )
        try:
            self.robot.move(movement, asynchronous=True)
        except Exception as e:
            self.get_logger().error(f"cartesian velocity move failed: {str(e)}")
            self.robot.recover_from_errors()

    def move_joints_sync_callback(
        self, request: BlockingJointMove.Request, response: BlockingJointMove.Response
    ):
        """Moves to target joint positions and blocks until movement is complete."""
        movement = franky.JointMotion(
            target=list(request.target.positions),
            reference_type=(
                franky.ReferenceType.Relative
                if request.target.relative
                else franky.ReferenceType.Absolute
            ),
        )
        self.get_logger().info("Blocking Joint Move Command Received.")
        try:
            self.robot.move(movement, asynchronous=False)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Blocking joint move failed: {str(e)}")
            response.success = False
        return response

    def move_cartesian_sync_callback(
        self,
        request: BlockingCartesianMove.Request,
        response: BlockingCartesianMove.Response,
    ):
        """Moves to target Cartesian pose and blocks until movement is complete."""
        pos = [
            request.target.pose.position.x,
            request.target.pose.position.y,
            request.target.pose.position.z,
        ]
        quat = [
            request.target.pose.orientation.x,
            request.target.pose.orientation.y,
            request.target.pose.orientation.z,
            request.target.pose.orientation.w,
        ]
        movement = franky.CartesianMotion(
            franky.Affine(pos, quat),
            reference_type=(
                franky.ReferenceType.Relative
                if request.target.relative
                else franky.ReferenceType.Absolute
            ),
        )
        self.get_logger().info("Blocking Cartesian Move Command Received.")
        try:
            self.robot.move(movement, asynchronous=False)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Blocking cartesian move failed: {str(e)}")
            response.success = False
        return response

    # --- Gripper Callbacks ---
    def gripper_move_callback(self, msg: GripperMove):
        self.get_logger().info(
            f"Gripper: Moving to width {msg.width}m at speed {msg.speed}m/s"
        )
        self.gripper.move_async(msg.width, msg.speed)
        self.get_logger().info("after async")

    def gripper_grasp_callback(self, msg: GripperGrasp):
        self.get_logger().info(f"Gripper: Grasping at width {msg.width}m")
        self.gripper.grasp_async(
            msg.width, msg.speed, msg.force, msg.epsilon_inner, msg.epsilon_outer
        )

    def move_gripper_sync_callback(
        self,
        request: BlockingGripperMove.Request,
        response: BlockingGripperMove.Response,
    ):
        self.get_logger().info(
            f"Gripper: Blocking move to width {request.target.width}m at speed {request.target.speed}m/s"
        )
        try:
            success = self.gripper.move(request.target.width, request.target.speed)
            response.success = success
        except Exception as e:
            self.get_logger().error(f"Blocking gripper move failed: {str(e)}")
            response.success = False
        return response

    def grasp_gripper_sync_callback(
        self,
        request: BlockingGripperGrasp.Request,
        response: BlockingGripperGrasp.Response,
    ):
        self.get_logger().info(
            f"Gripper: Blocking grasp at width {request.target.width}m"
        )
        try:
            success = self.gripper.grasp(
                request.target.width,
                request.target.speed,
                request.target.force,
                request.target.epsilon_inner,
                request.target.epsilon_outer,
            )
            response.success = success
        except Exception as e:
            self.get_logger().error(f"Blocking gripper grasp failed: {str(e)}")
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FrankyRosBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.robot.stop()
        node.robot_web_session.release_control()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
