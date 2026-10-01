import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Point
from enum import Enum, auto
from trajectory_msgs.msg import JointTrajectory

EBSILON = 1e-6

class ControlState(Enum):
    INIT     = auto()
    MOVE_1   = auto()
    DIVE_1   = auto()
    ALIGN_1  = auto()
    REINIT   = auto()
    MOVE_2   = auto()
    DROP     = auto()

JOINT_NAMES = ["joint1", "joint2", "joint3","joint4", "joint5", "joint6"]


MOVE_SEC = 3 # very very important


class Lite6Control(Node):
    def __init__(self):
        super().__init__("lite6_control")
        
        self.current_state = ControlState.INIT
        self.cmd_data      = None
        self.cmd_sub       = self.create_subscription(String, '/cmd',         self._cb_cmd,    10)
        self.state_pub     = self.create_publisher(String,    '/robot_state', 10)
        self.timer         = self.create_timer(0.1, self._tick)

    def _cb_cmd(self, msg: String):
        self.cmd_data = msg.data

    def _tick(self):
        if self.current_state == ControlState.INIT:
            self._step_init()
        elif self.current_state == ControlState.MOVE_1:
            self._step_aruco_0()
        elif self.current_state == ControlState.DIVE_1:
            self._step_dive_1()
        elif self.current_state == ControlState.ALIGN_1:
            self._step_align_1()
        elif self.current_state == ControlState.REINIT:
            self._step_reinit()
        elif self.current_state == ControlState.MOVE_2:
            self._step_aruco_1()
        elif self.current_state == ControlState.DROP:
            self._step_drop()

    def _step_init(self):
        self.state_pub.publish(String(data=self.current_state.name))
        self.get_logger().info("→ INIT")
        if self.cmd_data == 'DONE':
            self.cmd_data = None
            self.current_state = ControlState.MOVE_1
            self.state_pub.publish(String(data=self.current_state.name))

    def _step_aruco_0(self):
        self.get_logger().info("→ MOVE_1")            
        if self.cmd_data == 'DONE':
            self.cmd_data = None
            self.current_state = ControlState.DIVE_1
            self.state_pub.publish(String(data=self.current_state.name))

    def _step_dive_1(self):
        self.get_logger().info("→ DIVE_1")
        if self.cmd_data == 'DONE':
            self.cmd_data = None
            self.current_state = ControlState.ALIGN_1
            self.state_pub.publish(String(data=self.current_state.name))

    def _step_align_1(self):
        self.get_logger().info("→ ALIGN_1")
        if self.cmd_data == 'DONE':
            self.cmd_data = None
            self.current_state = ControlState.REINIT
            self.state_pub.publish(String(data=self.current_state.name))

    def _step_reinit(self):
        self.get_logger().info("→ REINIT")
        if self.cmd_data == 'DONE':
            self.cmd_data = None
            self.current_state = ControlState.MOVE_2
            self.state_pub.publish(String(data=self.current_state.name))

    def _step_aruco_1(self):
        self.get_logger().info("→ MOVE_2")
        if self.cmd_data == 'DONE':
            self.cmd_data = None
            self.current_state = ControlState.DROP
            self.state_pub.publish(String(data=self.current_state.name))

    def _step_drop(self): 
        self.get_logger().info("→ DROP")
        if self.cmd_data == 'DONE':
            self.cmd_data = None
            self.current_state = ControlState.INIT
            self.state_pub.publish(String(data=self.current_state.name))


def main():
    rclpy.init()
    node = Lite6Control()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()