"""Isaac Sim Script Editor에서 실행:
Window → Script Editor → 이 코드 붙여넣고 Run
"""
import omni.usd
import os

os.makedirs("/home/v/xarm_ros2/xarm_description/urdf/lite6_complete", exist_ok=True)
stage = omni.usd.get_context().get_stage()
stage.Flatten().Export("/home/v/xarm_ros2/xarm_description/urdf/lite6_complete/lite6_complete.usda")
print("saved:", "/home/v/xarm_ros2/xarm_description/urdf/lite6_complete/lite6_complete.usda")
