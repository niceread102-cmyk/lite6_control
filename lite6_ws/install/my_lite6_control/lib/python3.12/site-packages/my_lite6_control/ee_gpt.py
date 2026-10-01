import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation


# x y z만 입력했을 때 사용할 기본값
# roll, pitch, local_z_spin
DEFAULT_ORIENTATION = (180.0, 0.0, 0.0)

# EE에서 손목 회전에 사용할 로컬 축
# 현재는 local Z축
LOCAL_SPIN_AXIS = np.array([0.0, 0.0, 1.0])


class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')

        self.target_pub = self.create_publisher(
            Pose,
            '/target_pose',
            10
        )

    @staticmethod
    def make_local_z_orientation(roll, pitch, local_z_spin):
        """
        1. roll, pitch로 EE 기본 기울기를 만든다.
        2. 기울어진 EE의 local Z축으로 local_z_spin만큼 회전한다.

        결과:
            R_final = R_tilt @ R_local_z
        """

        # yaw=0인 기본 기울기
        r_tilt = Rotation.from_euler(
            'xyz',
            [roll, pitch, 0.0],
            degrees=True
        )

        # 기울어진 EE의 local Z축 회전
        r_local_spin = Rotation.from_rotvec(
            np.deg2rad(local_z_spin) * LOCAL_SPIN_AXIS
        )

        # 오른쪽 곱이므로 local-frame 회전
        r_final = r_tilt * r_local_spin

        return r_final

    def publish_target(
        self,
        x,
        y,
        z,
        roll=None,
        pitch=None,
        local_z_spin=None
    ):
        if roll is None:
            roll, pitch, local_z_spin = DEFAULT_ORIENTATION

        values = np.array(
            [x, y, z, roll, pitch, local_z_spin],
            dtype=float
        )

        if not np.all(np.isfinite(values)):
            raise ValueError('Input contains NaN or infinity.')

        msg = Pose()

        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)

        r_final = self.make_local_z_orientation(
            roll,
            pitch,
            local_z_spin
        )

        # SciPy quaternion 순서: [x, y, z, w]
        qx, qy, qz, qw = r_final.as_quat()

        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.orientation.w = float(qw)

        self.target_pub.publish(msg)

        self.get_logger().info(
            f'Published local-Z target: '
            f'pos=[{x:.3f}, {y:.3f}, {z:.3f}] '
            f'roll={roll:.4f}, '
            f'pitch={pitch:.4f}, '
            f'local_z_spin={local_z_spin:.4f} '
            f'quat_xyzw=['
            f'{qx:.6f}, {qy:.6f}, '
            f'{qz:.6f}, {qw:.6f}]'
        )

    def publish_target_quat(
        self,
        x,
        y,
        z,
        qx,
        qy,
        qz,
        qw
    ):
        position = np.array([x, y, z], dtype=float)
        quaternion = np.array(
            [qx, qy, qz, qw],
            dtype=float
        )

        if not np.all(np.isfinite(position)):
            raise ValueError(
                'Position contains NaN or infinity.'
            )

        if not np.all(np.isfinite(quaternion)):
            raise ValueError(
                'Quaternion contains NaN or infinity.'
            )

        norm = np.linalg.norm(quaternion)

        if norm < 1e-12:
            raise ValueError(
                'Quaternion norm must not be zero.'
            )

        # quaternion 정규화
        quaternion /= norm
        qx, qy, qz, qw = quaternion

        msg = Pose()

        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)

        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.orientation.w = float(qw)

        self.target_pub.publish(msg)

        self.get_logger().info(
            f'Published quaternion target: '
            f'pos=[{x:.3f}, {y:.3f}, {z:.3f}] '
            f'quat_xyzw=['
            f'{qx:.6f}, {qy:.6f}, '
            f'{qz:.6f}, {qw:.6f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()

    prompt = (
        'input target:\n'
        '  x y z\n'
        '  x y z roll pitch local_z_spin\n'
        '  x y z qx qy qz qw\n'
        "(or type 'q' to quit): "
    )

    try:
        while rclpy.ok():
            user_input = input(prompt).strip()

            if user_input.lower() == 'q':
                break

            values = user_input.split()

            try:
                if len(values) == 3:
                    x, y, z = map(float, values)

                    node.publish_target(
                        x,
                        y,
                        z
                    )

                elif len(values) == 6:
                    (
                        x,
                        y,
                        z,
                        roll,
                        pitch,
                        local_z_spin
                    ) = map(float, values)

                    node.publish_target(
                        x,
                        y,
                        z,
                        roll,
                        pitch,
                        local_z_spin
                    )

                elif len(values) == 7:
                    (
                        x,
                        y,
                        z,
                        qx,
                        qy,
                        qz,
                        qw
                    ) = map(float, values)

                    node.publish_target_quat(
                        x,
                        y,
                        z,
                        qx,
                        qy,
                        qz,
                        qw
                    )

                else:
                    raise ValueError(
                        'Enter 3, 6, or 7 numbers.'
                    )

            except ValueError as error:
                print(f'Invalid input: {error}')

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()