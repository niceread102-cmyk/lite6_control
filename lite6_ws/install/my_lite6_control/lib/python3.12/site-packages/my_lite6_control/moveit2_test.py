import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK

from my_lite6_control.config import default_config_path, describe, load_cli_config, load_config
from my_lite6_control.motion import (build_ik_request, build_joint_goal, eef_target_from_tcp, ik_joint_targets,
                                     prepare_trajectory, to_seconds)

# Planning / joint-limit parameters: config/default.yaml (planning, robot.joint_limits, moveit2, ros).


class MoveitTargetNode(Node):
    def __init__(self, cfg=None):
        super().__init__('moveit2')
        self.cfg             = cfg if cfg is not None else load_config(default_config_path())
        ros                  = self.cfg.ros
        self.sub             = self.create_subscription(Pose, ros.target_pose_topic, self._cb_target_pose, 10)
        self.joint_state_sub = self.create_subscription(JointState, ros.joint_states_topic, self._cb_joint_state, 10)
        self.done_pub        = self.create_publisher(Bool, ros.done_topic, 10)
        self.traj_pub        = self.create_publisher(JointTrajectory, ros.trajectory_topic, 10)
        self._ik_client      = self.create_client(GetPositionIK, ros.compute_ik_service)
        self._move_client    = ActionClient(self, MoveGroup, ros.move_group_action)
        self._busy           = False
        self.current_joint_state = None
        self.get_logger().info(describe(self.cfg))
        self.get_logger().info(f'moveit2 ready — waiting for {ros.target_pose_topic}')

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
        if not self._ik_client.wait_for_service(timeout_sec=self.cfg.ros.wait_timeout_s):
            self.get_logger().error('compute_ik service not available')
            self._busy = False
            self.done_pub.publish(Bool(data=False))
            return

        target = eef_target_from_tcp(pose, self.cfg)
        ik_req = build_ik_request(target, self.current_joint_state, self.cfg)
        self._ik_client.call_async(ik_req).add_done_callback(self._on_ik_result)

    def _on_ik_result(self, future):
        joint_targets, err = ik_joint_targets(future.result(), self.cfg)
        if err:
            self.get_logger().error(f'[FAIL] {err}')
            self._busy = False
            self.done_pub.publish(Bool(data=False))
            return

        self._send_goal(joint_targets)

    def _send_goal(self, joint_targets):
        if not self._move_client.wait_for_server(timeout_sec=self.cfg.ros.wait_timeout_s):
            self.get_logger().error('MoveGroup not available')
            self._busy = False
            self.done_pub.publish(Bool(data=False))
            return

        goal_msg = build_joint_goal(joint_targets, self.cfg, plan_only=True)
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
            if not prepare_trajectory(traj, self.cfg, self.get_logger()):
                self._busy = False
                self.done_pub.publish(Bool(data=False))
                return
            traj.header.stamp = self.get_clock().now().to_msg()
            self.traj_pub.publish(traj)
            self.get_logger().info(f'[OK] Trajectory published ({len(traj.points)} points)')

            if traj.points:
                delay = to_seconds(traj.points[-1].time_from_start) + self.cfg.moveit2.exec_margin_s
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
    cfg = load_cli_config()  # --config <yaml> / dotted overrides after the executable name
    rclpy.init(args=args)
    node = MoveitTargetNode(cfg)
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
