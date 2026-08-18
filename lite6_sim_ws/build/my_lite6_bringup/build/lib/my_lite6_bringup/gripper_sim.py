"""그리퍼 서비스 시뮬레이터 — 별도 터미널에서 실행
ros2 service call /ufactory/open_lite6_gripper xarm_msgs/srv/Call "{}"
ros2 service call /ufactory/close_lite6_gripper xarm_msgs/srv/Call "{}"
"""
import rclpy
from rclpy.node import Node
from xarm_msgs.srv import Call
from std_msgs.msg import Float64

class GripperSim(Node):
    def __init__(self):
        super().__init__("gripper_sim")
        self._pub = self.create_publisher(Float64, "/gripper_target_m", 10)
        self.create_service(Call, "/ufactory/open_lite6_gripper",  lambda q, r: self._cb(q, r, 0.0089))
        self.create_service(Call, "/ufactory/close_lite6_gripper", lambda q, r: self._cb(q, r, 0.0))
        self.create_service(Call, "/ufactory/stop_lite6_gripper",  lambda q, r: self._cb(q, r, None))
        self.get_logger().info("gripper_sim ready")

    def _cb(self, req, res, val):
        if val is not None:
            msg = Float64(); msg.data = val
            self._pub.publish(msg)
        res.ret = 0; res.message = ""
        return res

rclpy.init()
rclpy.spin(GripperSim())
