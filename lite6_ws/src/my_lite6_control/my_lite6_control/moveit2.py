import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation
import numpy as np

JOINT_GOAL_TOLERANCE = 0.0005

EEF_LINK   = 'link_eef'
BASE_FRAME = 'link_base'
TCP_OFFSET = 0.0836

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def eef_target_from_tcp(pose: Pose) -> PoseStamped:
    R       = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w]).as_matrix()
    tcp_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    eef_pos = tcp_pos - R @ np.array([0.0, 0.0, TCP_OFFSET])

    target = PoseStamped()
    target.header.frame_id = BASE_FRAME
    target.pose.position.x, target.pose.position.y, target.pose.position.z = eef_pos.tolist()
    target.pose.orientation = pose.orientation
    return target


def build_ik_request(target: PoseStamped, joint_state: JointState) -> GetPositionIK.Request:
    # compute_ik seeded with the CURRENT joint state -> converges to the nearest
    # joint solution (Newton/Jacobian), instead of MoveGroup picking an arbitrary
    # branch inside a loose position/orientation tolerance region (the source of
    # the micro-jitter/wild-swing behaviour).
    req                                    = GetPositionIK.Request()
    req.ik_request.group_name              = 'lite6'
    req.ik_request.robot_state.joint_state = joint_state
    req.ik_request.avoid_collisions        = True
    req.ik_request.ik_link_name            = EEF_LINK
    req.ik_request.pose_stamped            = target
    req.ik_request.timeout                 = Duration(sec=1)
    return req


class MoveitTargetNode(Node):
    def __init__(self):
        super().__init__('moveit2')
        self.sub             = self.create_subscription(Pose, '/target_pose', self._cb_target_pose, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self._cb_joint_state, 10)
        self.done_pub        = self.create_publisher(Bool, '/ik_done', 10)
        self.traj_pub        = self.create_publisher(JointTrajectory, '/lite6_traj_controller/joint_trajectory', 10)
        self._ik_client      = self.create_client(GetPositionIK, '/compute_ik')
        self._move_client    = ActionClient(self, MoveGroup, '/move_action')
        self._busy           = False
        self.current_joint_state = None
        self.get_logger().info('moveit2 ready — waiting for /target_pose')

    def _cb_joint_state(self, msg: JointState):
        self.current_joint_state = msg

    def _cb_target_pose(self, msg: Pose):
        if self._busy:
            self.get_logger().warn('Already moving, ignoring')
            return

        if self.current_joint_state is None:
            self.get_logger().warn('No /joint_states yet, ignoring target')
            return

        self._busy = True
        self._send_ik_request(msg)

    def _send_ik_request(self, pose: Pose):
        if not self._ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('compute_ik service not available')
            self._busy = False
            self.done_pub.publish(Bool(data=False))
            return

        target = eef_target_from_tcp(pose)
        ik_req = build_ik_request(target, self.current_joint_state)
        self._ik_client.call_async(ik_req).add_done_callback(self._on_ik_result)

    def _on_ik_result(self, future):
        response = future.result()
        if response.error_code.val != 1:
            self.get_logger().error(f'[FAIL] compute_ik error_code={response.error_code.val}')
            self._busy = False
            self.done_pub.publish(Bool(data=False))
            return

        solved = dict(zip(response.solution.joint_state.name,
                           response.solution.joint_state.position))
        try:
            joint_targets = [solved[n] for n in JOINT_NAMES]
        except KeyError:
            self.get_logger().error('IK solution missing expected joints')
            self._busy = False
            self.done_pub.publish(Bool(data=False))
            return

        self._send_goal(joint_targets)

    def _send_goal(self, joint_targets):
        if not self._move_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('MoveGroup not available')
            self._busy = False
            self.done_pub.publish(Bool(data=False))
            return

        joint_con = [JointConstraint(joint_name=n, position=p,
                                      tolerance_above=JOINT_GOAL_TOLERANCE,
                                      tolerance_below=JOINT_GOAL_TOLERANCE, weight=1.0)
                     for n, p in zip(JOINT_NAMES, joint_targets)]

        req                                  = MotionPlanRequest()
        req.group_name                       = 'lite6'
        req.planner_id                       = 'RRTConnect'
        req.start_state.is_diff              = True
        req.goal_constraints                 = [Constraints(joint_constraints=joint_con)]
        req.num_planning_attempts            = 10
        req.allowed_planning_time            = 5.0
        req.max_velocity_scaling_factor      = 0.15
        req.max_acceleration_scaling_factor  = 0.05

        goal_msg                             = MoveGroup.Goal()
        goal_msg.request                     = req
        goal_msg.planning_options.plan_only  = True

        self._move_client.send_goal_async(goal_msg).add_done_callback(self._goal_cb)

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
            traj = result.planned_trajectory.joint_trajectory
            traj.header.stamp = self.get_clock().now().to_msg()
            self.traj_pub.publish(traj)
            self.get_logger().info(f'[OK] Trajectory published ({len(traj.points)} points)')

            if traj.points:
                t = traj.points[-1].time_from_start
                delay = t.sec + t.nanosec * 1e-9 + 0.5
            else:
                delay = 1.0
            self._exec_wait_timer = self.create_timer(delay, self._on_traj_exec_done)
        else:
            self.get_logger().error(
                f'[FAIL] error_code={code} message="{result.error_code.message}" '
                f'source="{result.error_code.source}"')
            self._busy = False
            self.done_pub.publish(Bool(data=False))

    def _on_traj_exec_done(self):
        self._exec_wait_timer.cancel()
        self._busy = False
        self.done_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = MoveitTargetNode()
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
