import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Point, Pose
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

APPROACH_Z  = 0.15
DIVE_OFFSET = 0.10
EEF_LINK    = "link_eef"
BASE_FRAME  = "link_base"
CENTERED    = 15

# 제어 파라미터
PIXEL_TOLERANCE = 10.0      # 오차 허용 범위 (픽셀)
STEP_SIZE_M     = 0.01      # 한 번 플래닝할 때 이동할 최대 거리 (미터) - 너무 크면 위험, 너무 작으면 느림
PIXEL_TO_METER  = 0.0005    # 픽셀 오차를 미터로 변환하는 대략적인 비율 (카메라 높이에 따라 튜닝 필수)

 
JOINT_NAMES = ["joint1", "joint2", "joint3","joint4", "joint5", "joint6"]
INIT_POS    = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
REINIT_POS  = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]

class Moveit2(Node):
    def __init__(self):
        super().__init__('moveit2_picker')

        self.aruco_center_sub = self.create_subscription(Point,        '/aruco_center', self._cb_aruco_center,    10)
        self.frame_center_sub = self.create_subscription(Point,        '/frame_center', self._cb_frame_center,    10)
        self.ee_pose_sub      = self.create_subscription(Pose,         '/ee_pose',      self._cb_ee_pose,         10)
        self.traj_pub         = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 10)
        self.cmd_pub          = self.create_publisher(String,          '/cmd',                                    10)
        self.cmd_data         = None

        self.state_sub        = self.create_subscription(String,       '/robot_state', self._cb_state, 10)
        self._move_client     = ActionClient(self, MoveGroup, '/move_action')
        
        self.gear_sub  = self.create_subscription(Point, '/gear_center', self._cb_dect_obj ,10)


        self.aruco_center = None
        self.frame_center = None
        self.frame_loss   = None
        self.ee_pose      = None

        self.is_moving   = False
        self.is_centered = False
        
        self.current_state = None

        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info("MoveIt2 Picker Ready. Waiting for ArUco...")

        self.obj   = None

        self.x  = 0.0
        self.y  = 0.0
        self.z  = 0.0

        self.qw = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0



    def _cb_aruco_center(self, msg: Point):
        self.aruco_center = msg


    def _cb_frame_center(self, msg: Point):
        self.frame_center = msg


    def _cb_ee_pose(self, msg: Pose):
        self.ee_pose = msg

        self.x = self.ee_pose.position.x
        self.y = self.ee_pose.position.y
        self.z = self.ee_pose.position.z

        self.qw = self.ee_pose.orientation.w
        self.qx = self.ee_pose.orientation.x
        self.qy = self.ee_pose.orientation.y
        self.qz = self.ee_pose.orientation.z
                        

    def _cb_state(self, msg: String):
        self.current_state = msg.data
        self.get_logger().info(f"state: {self.current_state}")


    def _send_done(self):
        self._done_timer.cancel()
        self.is_moving = False
        self.current_state = None
        self.cmd_pub.publish(String(data="DONE"))

    def _cb_dect_obj(self, msg: Point):
        self.obj = msg


    def _send_goal(self, target: PoseStamped):
        if not self._move_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("MoveGroup not available")
            self.is_moving = False
            return

        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])
        bv     = BoundingVolume(primitives=[sphere], primitive_poses=[target.pose])

        pos_con = PositionConstraint()
        pos_con.header            = target.header
        pos_con.link_name         = EEF_LINK
        pos_con.constraint_region = bv
        pos_con.weight            = 1.0

        ori_con = OrientationConstraint()
        ori_con.header                    = target.header
        ori_con.link_name                 = EEF_LINK
        ori_con.orientation               = target.pose.orientation
        ori_con.absolute_x_axis_tolerance = 0.1
        ori_con.absolute_y_axis_tolerance = 0.1
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight                    = 1.0

        req = MotionPlanRequest()
        req.group_name              = "lite6"
        req.goal_constraints        = [Constraints(
            position_constraints    = [pos_con],
            orientation_constraints = [ori_con],
        )]
        req.num_planning_attempts           = 5
        req.allowed_planning_time           = 2.0
        req.max_velocity_scaling_factor     = 0.3
        req.max_acceleration_scaling_factor = 0.1

        goal_msg = MoveGroup.Goal()
        goal_msg.request                    = req
        goal_msg.planning_options.plan_only = False

        self.is_moving = True
        self._move_client.send_goal_async(goal_msg).add_done_callback(self._goal_cb)

    def _goal_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal rejected")
            self.is_moving = False
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        code = future.result().result.error_code.val
        self.get_logger().info(f"Move done  error_code={code}")
        self.is_moving = False

    def _publish_pos(self, pos):
        pt = JointTrajectoryPoint()
        pt.positions       = list(pos)
        pt.time_from_start = Duration(sec=3)

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = JOINT_NAMES
        msg.points       = [pt]
        self.traj_pub.publish(msg)
    

    def _control_loop(self):
        if self.current_state == 'INIT':
            if not self.is_moving:
                self.is_moving = True
                self._publish_pos(INIT_POS)
                self._done_timer = self.create_timer(3.5, self._send_done)

        elif self.current_state == 'MOVE_1':
            if self.aruco_center is None or self.frame_center is None or self.ee_pose is None:
                return
            if self.is_moving:
                return

            dx = self.frame_center.x - self.aruco_center.x
            dy = self.frame_center.y - self.aruco_center.y
            self.get_logger().info(f"pixel error  dx={dx:.1f}  dy={dy:.1f}")

            if math.hypot(dx, dy) < CENTERED:
                self.get_logger().info("Centered! → DONE")
                self.cmd_pub.publish(String(data="DONE"))

                self.current_state = 'DIVE_1'
                return

            move_x = dx * PIXEL_TO_METER
            move_y = dy * PIXEL_TO_METER
            scale  = min(1.0, STEP_SIZE_M / (math.hypot(move_x, move_y) + 1e-9))
            move_x *= scale
            move_y *= scale

            target = PoseStamped()
            target.header.frame_id    = BASE_FRAME
            target.header.stamp       = self.get_clock().now().to_msg()
            target.pose.position.x    = self.x + move_y
            target.pose.position.y    = self.y + move_x
            target.pose.position.z    = self.z
            target.pose.orientation.x = 1.0
            target.pose.orientation.y = 0.0
            target.pose.orientation.z = 0.0
            target.pose.orientation.w = 0.0
            self._send_goal(target)

        elif self.current_state == 'DIVE_1':
            x, y = self.obj.x, self.obj.y

            dx = self.frame_center.x - x
            dy = self.frame_center.y - y
            
            
        

        


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