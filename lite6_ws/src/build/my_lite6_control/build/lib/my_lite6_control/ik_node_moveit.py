import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints,
                              PositionConstraint, OrientationConstraint,
                              BoundingVolume)
from shape_msgs.msg import SolidPrimitive
from scipy.spatial.transform import Rotation
import numpy as np

EEF_LINK   = 'link_eef'
BASE_FRAME = 'link_base'
TCP_OFFSET = 0.0836


def build_move_goal(pose: Pose, vel_scale=0.15, acc_scale=0.05):
    R       = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w]).as_matrix()
    tcp_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    eef_pos = tcp_pos - R @ np.array([0.0, 0.0, TCP_OFFSET])

    target = PoseStamped()
    target.header.frame_id = BASE_FRAME
    target.pose.position.x, target.pose.position.y, target.pose.position.z = eef_pos.tolist()
    target.pose.orientation = pose.orientation

    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])
    bv     = BoundingVolume(primitives=[sphere], primitive_poses=[target.pose])

    pos_con                   = PositionConstraint()
    pos_con.header            = target.header
    pos_con.link_name         = EEF_LINK
    pos_con.constraint_region = bv
    pos_con.weight            = 1.0

    ori_con                             = OrientationConstraint()
    ori_con.header                      = target.header
    ori_con.link_name                   = EEF_LINK
    ori_con.orientation                 = target.pose.orientation
    ori_con.absolute_x_axis_tolerance   = 0.05
    ori_con.absolute_y_axis_tolerance   = 0.05
    ori_con.absolute_z_axis_tolerance   = 0.05
    ori_con.weight                      = 1.0

    req                                 = MotionPlanRequest()
    req.group_name                      = 'lite6'
    req.goal_constraints                = [Constraints(
        position_constraints    = [pos_con],
        orientation_constraints = [ori_con],
    )]
    req.num_planning_attempts           = 10
    req.allowed_planning_time           = 5.0
    req.max_velocity_scaling_factor     = vel_scale
    req.max_acceleration_scaling_factor = acc_scale

    goal_msg                             = MoveGroup.Goal()
    goal_msg.request                     = req
    goal_msg.planning_options.plan_only  = False
    return goal_msg


class IK_Node_Moveit(Node):
    def __init__(self):
        super().__init__('IK_Node_Moveit')
        self.sub      = self.create_subscription(Pose, '/target_pose', self._cb_target_pose, 10)
        self.done_pub = self.create_publisher(Bool, '/ik_done', 10)
        self._client  = ActionClient(self, MoveGroup, '/move_action')
        self._busy    = False
        self.get_logger().info('IK_Node_Moveit ready — waiting for /target_pose')

    def _cb_target_pose(self, msg: Pose):
        if self._busy:
            self.get_logger().warn('Already moving, ignoring')
            return

        self._busy = True
        self._send_goal(msg)

    def _send_goal(self, pose: Pose):
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('MoveGroup not available')
            self._busy = False
            self.done_pub.publish(Bool(data=False))
            return

        goal_msg = build_move_goal(pose)
        self._client.send_goal_async(goal_msg).add_done_callback(self._goal_cb)

    def _goal_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected')
            self._busy = False
            self.done_pub.publish(Bool(data=False))
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        code   = result.error_code.val
        ok     = (code == 1)
        if ok:
            self.get_logger().info('[OK] Move completed')
        else:
            self.get_logger().error(
                f'[FAIL] error_code={code} message="{result.error_code.message}" '
                f'source="{result.error_code.source}"')
        self._busy = False
        self.done_pub.publish(Bool(data=ok))


def main(args=None):
    rclpy.init(args=args)
    node = IK_Node_Moveit()
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
