from rclpy.node import Node
from geometry_msgs.msg import Pose

class RobotData(Node):
    def __init__(self):
        super().__init__("robot_data")

        self.ee_pos  = [0.0, 0.0, 0.0]
        self.ee_quat = [0.0, 0.0, 0.0, 0.0]

        self.ee_pose_sub = self.create_subscription(Pose, "/ee_pose", self._cb_ee_pose, 10)

    def _cb_ee_pose(self, msg: Pose):
        self.ee_pos[0]  = msg.position.x
        self.ee_pos[1]  = msg.position.y
        self.ee_pos[2]  = msg.position.z

        self.ee_quat[0] = msg.orientation.x
        self.ee_quat[1] = msg.orientation.y
        self.ee_quat[2] = msg.orientation.z
        self.ee_quat[3] = msg.orientation.w
