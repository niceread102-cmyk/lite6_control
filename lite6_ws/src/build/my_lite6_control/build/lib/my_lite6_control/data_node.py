import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import custom_ik_rotation

class IK_Calculate(Node):
    def __init__(self):
        super().__init__('IK_Calculate')

        self.sub = self.create_subscription(Pose, '/target_pose', self._cb_target_pose, 10)
        self.pub = self.create_publisher(JointTrajectory, '/sim_lite6_traj_controller/joint_trajectory', 10)
        self.theta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def _cb_target_pose(self, msg: Pose):
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        threshold = np.sqrt(x**2 + y**2 + z**2)

        target_pos = np.array([x, y, z])
        target_R   = quat_to_R(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

        

def main():
    rclpy.init()
    node = IK_Calculate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()