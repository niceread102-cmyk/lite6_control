import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation
import subprocess
import threading
import time


HOME     = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
TARGET   = (0.25,  0.16,  0.13, 180.0,   0.0, 90.0)
TARGET_2 = (0.25, -0.05,  0.13, 135.0,   0.0,  0.0)
DROP     = (0.28, -0.30,  0.20, 180.0,   0.0,  0.0)

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class SequenceNode(Node):
    def __init__(self):
        super().__init__('sequence_node')
        self.pose_pub  = self.create_publisher(Pose, '/target_pose', 10)
        self.traj_pub  = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 10)
        self.done_sub  = self.create_subscription(Bool, '/ik_done', self._cb_done, 10)
        self._done_evt = threading.Event()

    def _cb_done(self, msg: Bool):
        self._done_evt.set()

    def send_home(self, move_sec=4.0):
        pt = JointTrajectoryPoint()
        pt.positions = HOME
        pt.time_from_start = Duration(sec=int(move_sec))
        jt = JointTrajectory()
        jt.header.stamp = self.get_clock().now().to_msg()
        jt.joint_names = JOINT_NAMES
        jt.points = [pt]
        self.traj_pub.publish(jt)
        self.get_logger().info('home')
        time.sleep(move_sec + 1.0)

    def send_pose(self, x, y, z, roll, pitch, yaw, timeout=30.0):
        self._done_evt.clear()
        q = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = x, y, z
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        self.pose_pub.publish(msg)
        self.get_logger().info(f'pose ({x}, {y}, {z}) rpy ({roll}, {pitch}, {yaw})')
        if not self._done_evt.wait(timeout=timeout):
            self.get_logger().error(f'send_pose timeout after {timeout}s')

    def call_gripper(self, srv_name):
        subprocess.run(
            f'source /home/v/xarm_ros2/install/setup.bash && ros2 service call {srv_name} xarm_msgs/srv/Call "{{}}"',
            shell=True, executable='/bin/bash'
        )
        time.sleep(1.5)

    def run(self):
        time.sleep(1.0)
        self.call_gripper('/ufactory/open_lite6_gripper')

        self.send_home()
        self.send_pose(*TARGET)
        self.call_gripper('/ufactory/close_lite6_gripper')

        self.send_home()
        self.send_pose(*DROP)
        self.call_gripper('/ufactory/open_lite6_gripper')

        self.send_home()
        self.send_pose(*TARGET_2)
        self.call_gripper('/ufactory/close_lite6_gripper')

        self.send_home()
        self.send_pose(*DROP)
        self.call_gripper('/ufactory/open_lite6_gripper')

        self.send_home()
        self.get_logger().info('sequence done')


def main(args=None):
    rclpy.init(args=args)
    node = SequenceNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.run()

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
