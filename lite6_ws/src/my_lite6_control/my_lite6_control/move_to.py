"""Interactive target input in any TF frame -> /target_pose (TCP pose in robot.base_frame).

Input line: [frame] x y z [roll pitch yaw (deg, intrinsic XYZ) | qx qy qz qw]
  frame = an alias from move_to.frames (base, cam, tcp, ...) or any TF frame name; omitted = move_to.frame.
  The pose is the TCP pose expressed in that frame. With only x y z the orientation comes from
  move_to.xyz_orientation (down = tool straight down in the base frame, current = keep the TCP orientation).
The frame -> base transform is looked up when the line is entered, so for the wrist camera the arm must be where
it was when the camera measured the point. Run a consumer of /target_pose (moveit2, sequence_real_action, ...).

    ros2 run my_lite6_control move_to
    ros2 run my_lite6_control move_to move_to.frame=cam move_to.xyz_orientation=current
"""
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformException, TransformListener

from my_lite6_control.config import describe, load_cli_config
from my_lite6_control.motion import pose_is_safe

DOWN_RPY_DEG = (180.0, 0.0, 0.0)  # tool z straight down in the base frame (same default as ee_6d)


class MoveToNode(Node):
    def __init__(self, cfg):
        super().__init__('move_to')
        self.cfg         = cfg
        self.mcfg        = cfg.move_to
        self.base        = cfg.robot.base_frame
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_pub  = self.create_publisher(Pose, cfg.ros.target_pose_topic, 10)
        self.get_logger().info(describe(cfg))

    def frame_name(self, name: str) -> str:
        return (self.mcfg.frames or {}).get(name, name)

    def lookup(self, target: str, source: str):
        """(R, t) of `source` in `target` (latest TF)."""
        tf = self.tf_buffer.lookup_transform(target, source, Time(),
                                             timeout=Duration(seconds=float(self.mcfg.tf_timeout_s)))
        q, p = tf.transform.rotation, tf.transform.translation
        return Rotation.from_quat([q.x, q.y, q.z, q.w]), np.array([p.x, p.y, p.z])

    def to_base(self, frame: str, values: list) -> Pose:
        R_bf, t_bf = self.lookup(self.base, frame)
        pos = R_bf.apply(values[:3]) + t_bf
        if len(values) == 3:
            if self.mcfg.xyz_orientation == 'current':
                rot, _ = self.lookup(self.base, self.cfg.robot.eef_link)  # TCP has the eef_link orientation
            else:
                rot = Rotation.from_euler('XYZ', DOWN_RPY_DEG, degrees=True)
        elif len(values) == 6:
            rot = R_bf * Rotation.from_euler('XYZ', values[3:], degrees=True)
        else:
            rot = R_bf * Rotation.from_quat(values[3:])

        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = pos.tolist()
        q = rot.as_quat()
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q.tolist()
        return msg

    def handle_line(self, line: str):
        words = line.split()
        frame = self.frame_name(self.mcfg.frame)
        if words and not _is_float(words[0]):
            frame = self.frame_name(words.pop(0))
        if len(words) not in (3, 6, 7) or not all(_is_float(w) for w in words):
            print('Invalid input. [frame] x y z | x y z roll pitch yaw | x y z qx qy qz qw')
            return
        try:
            msg = self.to_base(frame, [float(w) for w in words])
        except TransformException as e:
            self.get_logger().error(f'no TF {self.base} <- {frame}: {e}')
            return

        p, q = msg.position, msg.orientation
        rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('XYZ', degrees=True)
        self.get_logger().info(f'{frame} -> {self.base}: pos=[{p.x:.4f}, {p.y:.4f}, {p.z:.4f}] '
                               f'rpy=[{rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}]')
        if pose_is_safe(msg, self.get_logger(), self.cfg):
            self.target_pub.publish(msg)


def _is_float(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False


def main(args=None):
    cfg = load_cli_config()  # --config <yaml> / dotted overrides after the executable name
    rclpy.init(args=args)
    node = MoveToNode(cfg)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)  # TF listener runs while input() blocks
    spin.start()
    frames = ', '.join(f'{k}={v}' for k, v in (cfg.move_to.frames or {}).items())
    try:
        while rclpy.ok():
            line = input(f'[frame] x y z [roll pitch yaw | qx qy qz qw] (default frame {cfg.move_to.frame}; '
                         f'{frames}; q = quit): ').strip()
            if line.lower() == 'q':
                break
            if line:
                node.handle_line(line)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        rclpy.try_shutdown()  # stop the spin thread before destroying the node it spins
        spin.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
