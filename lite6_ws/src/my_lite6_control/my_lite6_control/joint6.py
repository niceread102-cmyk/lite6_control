import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class Joint6Publisher(Node):
    def __init__(self):
        super().__init__('joint6_publisher')
        self.sub_js  = self.create_subscription(JointState, '/joint_states', self._cb_joint_states, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 10)
        self.theta = None

    def _cb_joint_states(self, msg: JointState):
        try:
            self.theta = np.array([msg.position[msg.name.index(n)] for n in JOINT_NAMES])
        except ValueError:
            pass

    def publish_joint6(self, angle_deg):
        if self.theta is None:
            self.get_logger().error('No /joint_states received yet')
            return

        theta = self.theta.copy()
        theta[5] = np.deg2rad(angle_deg)

        move_sec = max(1.0, abs(theta[5] - self.theta[5]) * 2.0)

        pt = JointTrajectoryPoint()
        pt.positions = theta.tolist()
        pt.time_from_start = Duration(sec=int(move_sec), nanosec=int((move_sec % 1) * 1e9))

        jt = JointTrajectory()
        jt.header.stamp = self.get_clock().now().to_msg()
        jt.joint_names = JOINT_NAMES
        jt.points = [pt]
        self.traj_pub.publish(jt)
        self.get_logger().info(f'Published joint6={angle_deg:.1f} deg, theta={np.round(theta,3)}')


def main(args=None):
    rclpy.init(args=args)
    node = Joint6Publisher()

    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            user_input = input("input joint6 angle in degrees (or type 'q' to quit): ")

            if user_input.lower() == 'q':
                break

            try:
                angle_deg = float(user_input)
                node.publish_joint6(angle_deg)
            except ValueError:
                print("Invalid input. Enter a single number (degrees).")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
