import numpy as np
from scipy.spatial.transform import Rotation


def to_matrix(x, y, z, roll, pitch, yaw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('XYZ', [roll, pitch, yaw], degrees=True).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def from_matrix(T):
    pos = T[:3, 3]
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler('XYZ', degrees=True)
    return pos, rpy


def main():
    lx, ly, lz, lr, lp, lyaw = map(float, input().split())
    T_local = to_matrix(lx, ly, lz, lr, lp, lyaw)

    while True:
        line = input()
        if line.strip().lower() == 'q':
            break
        x, y, z, roll, pitch, yaw = map(float, line.split())
        T_obj = to_matrix(x, y, z, roll, pitch, yaw)
        pos, rpy = from_matrix(T_obj @ T_local)
        print(f"{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} {rpy[0]:.2f} {rpy[1]:.2f} {rpy[2]:.2f}")


if __name__ == '__main__':
    main()
