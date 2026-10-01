import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class JointViewer(Node):
    def __init__(self):
        super().__init__('view_joint')
        self.sub = self.create_subscription(JointState, '/joint_states', self._cb, 10)

    def _cb(self, msg: JointState):
        by_name = dict(zip(msg.name, msg.position))
        try:
            values = [by_name[n] for n in JOINT_NAMES]
        except KeyError:
            return  # /joint_states not fully populated yet

        rad_str = ' '.join(f'{v:+.5f}' for v in values)
        deg_str = ' '.join(f'{np.degrees(v):+7.3f}' for v in values)
        print(f'\rjoint[rad]: {rad_str}  |  joint[deg]: {deg_str}', end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = JointViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
