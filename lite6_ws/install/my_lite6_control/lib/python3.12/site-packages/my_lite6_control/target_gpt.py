import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener


# Isaac Sim / USD rotateXYZ와 동일하게 사용
# R = Rx(x) @ Ry(y) @ Rz(z)
EULER_ORDER = 'XYZ'

# 기준 상태
OBJ_INIT_XYZ = np.array([-90.0, 0.0, 90.0])
EE_INIT_XYZ = np.array([180.0, 0.0, 0.0])

BASE_FRAME = 'link_base'
EEF_FRAME = 'link_eef'


def calculate_gripper_rotation(obj_xyz_deg):
    """
    물체 local X축을 기어의 법선축으로 사용한다.

    기준 물체 자세:
        OBJ XYZ = (-90, 0, 90)
        object local X-axis = base -Z 방향

    기준 EE 자세:
        EE XYZ = (180, 0, 0)

    물체 법선의 XZ 평면 기울기만 EE pitch에 반영한다.
    """

    obj_xyz_deg = np.asarray(obj_xyz_deg, dtype=float)

    # Isaac Sim/USD XYZ 순서
    R_obj = Rotation.from_euler(
        EULER_ORDER,
        obj_xyz_deg,
        degrees=True
    ).as_matrix()

    # 기어의 법선축 = object local X-axis = 회전행렬 첫 번째 열
    normal = R_obj[:, 0]

    nx = normal[0]
    ny = normal[1]
    nz = normal[2]

    # XZ 평면에 투영했을 때 방향이 정의되지 않는 경우
    if np.hypot(nx, nz) < 1e-8:
        raise ValueError(
            'Object normal is nearly parallel to the base Y-axis. '
            'EE pitch cannot be determined from the XZ projection.'
        )

    # 기준 법선 (0, 0, -1) 대비 Base Y축 signed tilt
    #
    # Ry(theta) @ [0, 0, -1]
    #     = [-sin(theta), 0, -cos(theta)]
    #
    # 따라서:
    # theta = atan2(-nx, -nz)
    grip_pitch_deg = np.degrees(
        np.arctan2(-nx, -nz)
    )

    # -0.000 같은 출력 제거
    if abs(grip_pitch_deg) < 1e-9:
        grip_pitch_deg = 0.0

    # 그리퍼는 항상 바닥을 보는 기본 자세를 유지하고
    # Y축 기울기만 물체 법선에 따라 변경
    ee_xyz_deg = np.array([
        EE_INIT_XYZ[0],   # 180°
        grip_pitch_deg,
        EE_INIT_XYZ[2],   # 0°
    ])

    R_ee_target = Rotation.from_euler(
        EULER_ORDER,
        ee_xyz_deg,
        degrees=True
    ).as_matrix()

    return R_ee_target, ee_xyz_deg, normal


class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_new_publisher')

        self.target_pub = self.create_publisher(
            Pose,
            '/target_pose',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.check_reference_pose()

    def check_reference_pose(self):
        """
        기준 OBJ 자세를 넣었을 때
        EE가 (180, 0, 0)으로 나오는지 시작 시 검산한다.
        """
        R_target, ee_xyz, normal = calculate_gripper_rotation(
            OBJ_INIT_XYZ
        )

        self.get_logger().info(
            'Reference-pose check: '
            f'OBJ XYZ={OBJ_INIT_XYZ.tolist()} -> '
            f'normal=[{normal[0]:.3f}, '
            f'{normal[1]:.3f}, '
            f'{normal[2]:.3f}] -> '
            f'EE XYZ=[{ee_xyz[0]:.3f}, '
            f'{ee_xyz[1]:.3f}, '
            f'{ee_xyz[2]:.3f}]'
        )

    def print_current_ee(self, timeout_sec=3.0):
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                EEF_FRAME,
                Time(),
                timeout=Duration(seconds=timeout_sec)
            )

        except Exception as e:
            self.get_logger().error(
                f'TF lookup {BASE_FRAME}->{EEF_FRAME} failed: {e}'
            )
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        xyz = np.array([
            t.x,
            t.y,
            t.z
        ])

        # Quaternion은 회전 순서와 무관하지만,
        # 출력 Euler 표현은 Isaac Sim/USD XYZ에 맞춘다.
        ee_xyz_deg = Rotation.from_quat([
            q.x,
            q.y,
            q.z,
            q.w
        ]).as_euler(
            EULER_ORDER,
            degrees=True
        )

        self.get_logger().info(
            f'current EE ({EEF_FRAME}) '
            f'xyz=[{xyz[0]:.3f}, '
            f'{xyz[1]:.3f}, '
            f'{xyz[2]:.3f}] '
            f'XYZ(deg)=[{ee_xyz_deg[0]:.3f}, '
            f'{ee_xyz_deg[1]:.3f}, '
            f'{ee_xyz_deg[2]:.3f}]'
        )

    def publish_target(
        self,
        x,
        y,
        z,
        obj_x,
        obj_y,
        obj_z
    ):
        """
        입력:
            x y z object_X object_Y object_Z

        object_XYZ는 Isaac Sim/USD rotateXYZ 기준.
        """

        obj_xyz_deg = np.array([
            obj_x,
            obj_y,
            obj_z
        ])

        try:
            R_final, ee_xyz_deg, normal = (
                calculate_gripper_rotation(obj_xyz_deg)
            )

        except ValueError as e:
            self.get_logger().error(str(e))
            return

        # SciPy quaternion 순서: [x, y, z, w]
        q = Rotation.from_matrix(
            R_final
        ).as_quat()

        msg = Pose()

        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)

        msg.orientation.x = float(q[0])
        msg.orientation.y = float(q[1])
        msg.orientation.z = float(q[2])
        msg.orientation.w = float(q[3])

        self.target_pub.publish(msg)

        self.get_logger().info(
            f'tcp=[{x:.3f}, {y:.3f}, {z:.3f}] '
            f'OBJ XYZ=[{obj_x:.3f}, '
            f'{obj_y:.3f}, {obj_z:.3f}] '
            f'normal=[{normal[0]:.3f}, '
            f'{normal[1]:.3f}, '
            f'{normal[2]:.3f}] '
            f'EE XYZ=[{ee_xyz_deg[0]:.3f}, '
            f'{ee_xyz_deg[1]:.3f}, '
            f'{ee_xyz_deg[2]:.3f}] '
            f'quat(wxyz)=[{q[3]:.5f}, '
            f'{q[0]:.5f}, '
            f'{q[1]:.5f}, '
            f'{q[2]:.5f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )
    spin_thread.start()

    node.print_current_ee()

    try:
        while rclpy.ok():
            user_input = input(
                'input x y z object_X object_Y object_Z '
                "(or type 'q' to quit): "
            ).strip()

            if user_input.lower() == 'q':
                break

            try:
                values = list(
                    map(float, user_input.split())
                )

                if len(values) != 6:
                    raise ValueError

                node.publish_target(*values)

            except ValueError:
                print(
                    'Invalid input. '
                    'Please enter exactly 6 numbers.'
                )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()