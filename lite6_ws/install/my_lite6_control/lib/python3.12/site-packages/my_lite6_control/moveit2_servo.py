import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TwistStamped, Point, Pose
from std_msgs.msg import String, Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# camera offset
# xyz: [0.06746, -0.0175, 0.0237]
# rpy: [π, -π/2, 0]

# servo setup 
# ros2 service call /servo_server/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"

EEF_LINK   = "link_eef"
BASE_FRAME = "link_base"
CENTERED   = 15

PIXEL_TO_METER = 0.05
MAX_VEL        = 0.05

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
INIT_POS    = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
REINIT_POS  = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
DROP_POS    = [0.0, 0.0, 1.57, 0.0, 0.0,  0.0]

class Moveit2(Node):
    def __init__(self):
        super().__init__('moveit2_picker')

        self.aruco_id_sub     = self.create_subscription(Int32,  '/aruco/target_id', self._cb_aruco_id,  10)
        self.aruco_center_sub = self.create_subscription(Point,  '/aruco_center', self._cb_aruco_center, 10)
        self.frame_center_sub = self.create_subscription(Point,  '/frame_center', self._cb_frame_center, 10)
        self.ee_pose_sub      = self.create_subscription(Pose,   '/ee_pose',      self._cb_ee_pose,      10)
        self.gear_sub         = self.create_subscription(Point,  '/gear_center',  self._cb_dect_obj,     10)
        self.state_sub        = self.create_subscription(String, '/robot_state',  self._cb_state,        10)

        self.traj_pub      = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 10)
        self.servo_pub     = self.create_publisher(TwistStamped,    '/servo_server/delta_twist_cmds',          10)
        self.cmd_pub       = self.create_publisher(String,          '/cmd',                                    10)

        self.aruco_center = None
        self.frame_center = None
        self.ee_pose      = None
        self.obj          = None

        self.is_moving     = False
        self.current_state = None
        self._align_target = None

        self.x = self.y = self.z = 0.0
        self.qw = self.qx = self.qy = self.qz = 0.0

        self.timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info("Moveit2 Servo Ready.")

    def _cb_aruco_center(self, msg: Point):
        self.aruco_center = msg

    def _cb_frame_center(self, msg: Point):
        self.frame_center = msg

    def _cb_ee_pose(self, msg: Pose):
        self.ee_pose = msg
        self.x  = msg.position.x;    self.y  = msg.position.y;   self.z  = msg.position.z
        self.qw = msg.orientation.w; self.qx = msg.orientation.x
        self.qy = msg.orientation.y; self.qz = msg.orientation.z

    def _cb_state(self, msg: String):
        if msg.data != self.current_state:
            self.is_moving = False
            self._align_target = None
        self.current_state = msg.data
        self.get_logger().info(f"state: {self.current_state}")

    def _cb_aruco_id(self, msg: Int32):
        self.target_id = msg.data
        self.get_logger().info(f"target_id: {self.target_id}")

    def _cb_dect_obj(self, msg: Point):
        self.obj = msg

    def _send_done(self):
        self._done_timer.cancel()
        self.is_moving     = False
        self.current_state = None
        self.cmd_pub.publish(String(data="DONE"))

    def _publish_pos(self, pos):
        pt = JointTrajectoryPoint()
        pt.positions       = list(pos)
        pt.time_from_start = Duration(sec=3)
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = JOINT_NAMES
        msg.points       = [pt]
        self.traj_pub.publish(msg)

    def _publish_twist(self, vx: float, vy: float, vz: float = 0.0):
        twist = TwistStamped()
        twist.header.frame_id = BASE_FRAME
        twist.header.stamp    = self.get_clock().now().to_msg()
        twist.twist.linear.x  = vx
        twist.twist.linear.y  = vy
        twist.twist.linear.z  = vz
        self.servo_pub.publish(twist)

    def _servo_toward(self, dx: float, dy: float, vz: float = None):
        vx = dy * PIXEL_TO_METER
        vy = dx * PIXEL_TO_METER
        speed = math.hypot(vx, vy)
        if speed > MAX_VEL:
            vx = vx / speed * MAX_VEL
            vy = vy / speed * MAX_VEL
        if vz is None:
            vz = -0.1 if self.z > 0.4 else 0.0
        self._publish_twist(vx, vy, vz)

    def _control_loop(self):
        if self.current_state == 'INIT':
            if not self.is_moving:
                self.is_moving = True
                self._publish_pos(INIT_POS)
                self._done_timer = self.create_timer(3.5, self._send_done)


        elif self.current_state == 'MOVE_1':
            if self.aruco_center is None or self.frame_center is None:
                return
            dx = self.frame_center.x - self.aruco_center.x
            dy = self.frame_center.y - self.aruco_center.y
            self.get_logger().info(f"MOVE_1 dx={dx:.1f} dy={dy:.1f}")
            if math.hypot(dx, dy) < CENTERED:
                self._publish_twist(0.0, 0.0)
                if not self.is_moving:
                    self.is_moving = True
                    self.cmd_pub.publish(String(data="DONE"))
                return
            self._servo_toward(dx, dy)


        elif self.current_state == 'DIVE_1':
            if self.obj is None or self.frame_center is None:
                return
            dx = self.frame_center.x - self.obj.x
            dy = self.frame_center.y - self.obj.y
            self.get_logger().info(f"DIVE_1 dx={dx:.1f} dy={dy:.1f} z={self.z:.3f}")
            if self.z <= 0.2:
                self._publish_twist(0.0, 0.0)
                if not self.is_moving:
                    self.is_moving = True
                    self.cmd_pub.publish(String(data="DONE"))
                return
            if math.hypot(dx, dy) < CENTERED:
                self._publish_twist(0.0, 0.0, -0.05)  # Phase B: 수직 하강
            else:
                self._servo_toward(dx, dy, 0.0)       # Phase A: 수평 정렬


        elif self.current_state == 'ALIGN_1':
            if self._align_target is None:
                self._align_target = (self.x + 0.06, self.y + 0.033)
            tx, ty = self._align_target
            err_x = tx - self.x
            err_y = ty - self.y
            dist = math.hypot(err_x, err_y)
            self.get_logger().info(f"ALIGN_1 err_x={err_x:.4f} err_y={err_y:.4f} z={self.z:.3f}")
            if dist < 0.005:
                if self.z <= 0.14 and not self.is_moving:
                    self._publish_twist(0.0, 0.0)
                    self.is_moving = True
                    self.cmd_pub.publish(String(data="DONE"))
                elif not self.is_moving:
                    self._publish_twist(0.0, 0.0, -0.05)
                    
            else:
                speed = max(dist, 1e-6)
                vx = min(err_x / speed * MAX_VEL, MAX_VEL)
                vy = min(err_y / speed * MAX_VEL, MAX_VEL)
                self._publish_twist(vx, vy, 0.0)


        elif self.current_state == 'REINIT':
            if not self.is_moving:
                self.is_moving = True
                self._publish_pos(REINIT_POS)
                self._done_timer = self.create_timer(3.5, self._send_done)
                


        elif self.current_state == 'MOVE_2':
            if self.aruco_center is None or self.frame_center is None:
                return
            dx = self.frame_center.x - self.aruco_center.x
            dy = self.frame_center.y - self.aruco_center.y
            self.get_logger().info(f"MOVE_2 dx={dx:.1f} dy={dy:.1f}")
            if math.hypot(dx, dy) < CENTERED:
                self._publish_twist(0.0, 0.0)
                if not self.is_moving:
                    self.is_moving = True
                    self.cmd_pub.publish(String(data="DONE"))
                return
            self._servo_toward(dx, dy)


        elif self.current_state == 'DROP':
            if not self.is_moving:
                self.is_moving = True
                self._publish_pos(DROP_POS)
                self._done_timer = self.create_timer(3.5, self._send_done)            

def main():
    rclpy.init()
    node = Moveit2()
    executor = MultiThreadedExecutor()
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
