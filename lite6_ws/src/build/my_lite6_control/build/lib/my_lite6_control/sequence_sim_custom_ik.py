import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32
from scipy.spatial.transform import Rotation
from my_lite6_control import custom_ik_rotation
from enum import Enum, auto
import numpy as np


HOME          = [0.0, 0.0, np.pi / 2, 0.0, np.pi / 2, 0.0]
GOAL          = [-np.pi / 2, np.pi / 2, np.pi, 0.0, np.pi / 4, 0.0]
JOINT_NAMES   = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
GRIPPER_OPEN  = 0.0089
GRIPPER_CLOSE = 0.0
APPROACH_DIST = 0.05
DESCEND_DIST  = 0.03
TCP_OFFSET    = 0.0836


def pre_grasp_pose(msg: Pose, dist=APPROACH_DIST) -> Pose:
    R = Rotation.from_quat([msg.orientation.x, msg.orientation.y,
                             msg.orientation.z, msg.orientation.w]).as_matrix()
    approach_dir = R[:, 2]
    pos = np.array([msg.position.x, msg.position.y, msg.position.z]) - dist * approach_dir

    pre = Pose()
    pre.position.x, pre.position.y, pre.position.z = pos.tolist()
    pre.orientation = msg.orientation
    return pre


def descend_pose(msg: Pose, dist=DESCEND_DIST) -> Pose:
    R = Rotation.from_quat([msg.orientation.x, msg.orientation.y,
                             msg.orientation.z, msg.orientation.w]).as_matrix()
    approach_dir = R[:, 2]
    pos = np.array([msg.position.x, msg.position.y, msg.position.z]) + dist * approach_dir

    dsc = Pose()
    dsc.position.x, dsc.position.y, dsc.position.z = pos.tolist()
    dsc.orientation = msg.orientation
    return dsc


class ControlState(Enum):
    INIT  = auto()
    MOVE  = auto()
    DIVE  = auto()
    GRASP = auto()
    HOME  = auto()
    GOAL  = auto()
    DROP  = auto()


class SequenceCustomIKNode(Node):
    def __init__(self):
        super().__init__('sequence_custom_ik_node')
        self.traj_pub    = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(Float32, '/gripper/command', 10)
        self.pose_sub    = self.create_subscription(Pose, '/target_pose', self._cb_target_pose, 10)
        self.js_sub      = self.create_subscription(JointState, '/joint_states', self._cb_joint_states, 10)
        self._state      = ControlState.INIT
        self._grasp_pose = None
        self._theta      = np.array(HOME)

        self._startup_timer = self.create_timer(0.5, self._start_init)

    def _cb_joint_states(self, msg: JointState):
        try:
            positions = [msg.position[msg.name.index(n)] for n in JOINT_NAMES]
            self._theta = np.array(positions)
        except ValueError:
            pass

    def _start_init(self):
        self._startup_timer.cancel()
        self.open_gripper()
        self._move_joint(HOME, 4.0, lambda ok: self._on_init_done())

    def _on_init_done(self):
        self._state = ControlState.MOVE
        self.get_logger().info('[MOVE] waiting for target pose')

    def _cb_target_pose(self, msg: Pose):
        if self._state != ControlState.MOVE:
            self.get_logger().warn(f'State={self._state.name}, ignoring')
            return

        self._grasp_pose = msg
        pre_pose = pre_grasp_pose(msg)
        self.get_logger().info(f'[MOVE] pre-grasp pos=({pre_pose.position.x:.3f}, {pre_pose.position.y:.3f}, {pre_pose.position.z:.3f})')
        self._move_linear(pre_pose, 4.0, self._on_pre_grasp_done)

    def _on_pre_grasp_done(self, ok):
        if not ok:
            self._state = ControlState.MOVE
            return

        self._state = ControlState.DIVE
        self.get_logger().info('[DIVE] descending to grasp pose')
        self._move_linear(self._grasp_pose, 16.0, self._on_dive_done)

    def _on_dive_done(self, ok):
        if not ok:
            self._state = ControlState.MOVE
            return

        dsc_pose = descend_pose(self._grasp_pose)
        self.get_logger().info(f'[DIVE] descending {DESCEND_DIST}m along tilt axis')
        self._move_linear(dsc_pose, 2.0, self._on_descend_done)

    def _on_descend_done(self, ok):
        if not ok:
            self._state = ControlState.MOVE
            return

        self._state = ControlState.GRASP
        self.get_logger().info('[GRASP] closing gripper')
        self.close_gripper()
        self._grasp_timer = self.create_timer(1.0, self._after_grasp)

    def _after_grasp(self):
        self._grasp_timer.cancel()
        self._state = ControlState.HOME
        self.get_logger().info('[HOME] returning home')
        self._move_joint(HOME, 4.0, self._on_home_done)

    def _on_home_done(self, ok):
        self._state = ControlState.GOAL
        self.get_logger().info('[GOAL] moving to goal pose')
        self._move_joint(GOAL, 4.0, self._on_goal_done)

    def _on_goal_done(self, ok):
        self._state = ControlState.DROP
        self.get_logger().info('[DROP] opening gripper')
        self.open_gripper()
        self._drop_timer = self.create_timer(1.0, self._after_drop)

    def _after_drop(self):
        self._drop_timer.cancel()
        self._state = ControlState.HOME
        self.get_logger().info('[HOME] returning home')
        self._move_joint(HOME, 4.0, self._on_cycle_done)

    def _on_cycle_done(self, ok):
        self._state = ControlState.MOVE
        self.get_logger().info('[MOVE] waiting for target pose')

    def _move_joint(self, target_theta, duration, on_done):
        self._theta = np.array(target_theta)
        self._publish_trajectory(self._theta, duration)
        self._wait_and_call(duration, on_done)

    def _move_linear(self, pose: Pose, duration, on_done):
        R = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z, pose.orientation.w]).as_matrix()
        tcp_end = np.array([pose.position.x, pose.position.y, pose.position.z])
        eef_end = tcp_end - R @ np.array([0.0, 0.0, TCP_OFFSET])

        theta, ok = custom_ik_rotation.solve_ik(eef_end, R, self._theta)
        if not ok:
            self.get_logger().error('[MOVE] IK failed')
            self._state = ControlState.MOVE
            on_done(False)
            return

        self._theta = theta
        self._publish_trajectory(theta, duration)
        self._wait_and_call(duration, on_done)

    def _publish_trajectory(self, theta, duration):
        pt = JointTrajectoryPoint()
        pt.positions        = theta.tolist()
        pt.time_from_start  = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))

        jt = JointTrajectory()
        jt.header.stamp = self.get_clock().now().to_msg()
        jt.joint_names  = JOINT_NAMES
        jt.points       = [pt]
        self.traj_pub.publish(jt)
        self.get_logger().info('trajectory published (1 pt)')

    def _wait_and_call(self, duration, on_done):
        self._motion_timer = self.create_timer(duration + 0.5, lambda: self._on_motion_done(on_done))

    def _on_motion_done(self, on_done):
        self._motion_timer.cancel()
        on_done(True)

    def open_gripper(self):
        self.gripper_pub.publish(Float32(data=GRIPPER_OPEN))
        self.get_logger().info('gripper open')

    def close_gripper(self):
        self.gripper_pub.publish(Float32(data=GRIPPER_CLOSE))
        self.get_logger().info('gripper close')


def main(args=None):
    rclpy.init(args=args)
    node = SequenceCustomIKNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
