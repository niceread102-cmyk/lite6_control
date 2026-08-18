import cv2
import numpy as np

DICT_CANDIDATES = [
    ("DICT_4X4_50",         cv2.aruco.DICT_4X4_50),
    ("DICT_5X5_50",         cv2.aruco.DICT_5X5_50),
    ("DICT_6X6_50",         cv2.aruco.DICT_6X6_50),
    ("DICT_7X7_50",         cv2.aruco.DICT_7X7_50),
    ("DICT_ARUCO_ORIGINAL", cv2.aruco.DICT_ARUCO_ORIGINAL),
]

class Aruco:
    def __init__(self):
        par = cv2.aruco.DetectorParameters_create()

        par.adaptiveThreshWinSizeMin    = 3
        par.adaptiveThreshWinSizeMax    = 23
        par.adaptiveThreshWinSizeStep   = 10
        par.adaptiveThreshConstant      = 7
        par.minMarkerPerimeterRate      = 0.03
        par.polygonalApproxAccuracyRate = 0.05

        self.params = par
        self.dicts = [
            (name, cv2.aruco.getPredefinedDictionary(dict_id))
            for name, dict_id in DICT_CANDIDATES
        ]

    def detect(self, frame):
        gray = frame if len(frame.shape) == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        for _, aruco_dict in self.dicts:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=self.params)
            if ids is not None:
                seen, u_corners, u_ids = set(), [], []
                for c, i in zip(corners, ids.flatten()):
                    if i not in seen:
                        seen.add(i)
                        u_corners.append(c)
                        u_ids.append(i)
                ids = np.array(u_ids).reshape(-1, 1)
                cv2.aruco.drawDetectedMarkers(frame, u_corners, ids)
                return u_corners, ids

        return None, None

    def aruco_marker_center(self, corners, ids, frame, fy, fx, target_id):
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == target_id:
                c  = corners[i][0]
                mx = int(np.mean(c[:, 0]))
                my = int(np.mean(c[:, 1]))
                cx = fx // 2
                cy = fy // 2

                cv2.circle(frame, (mx, my), 5, (0, 255, 0), -1)
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                return mx, my, cx, cy

        return None
