# lite6_motion_control

xArm Lite6 로봇팔 pick-and-place 제어 워크스페이스 (실물 로봇 + Isaac Sim 시뮬레이션).

## 구조

```
lite6_ws/          실제 로봇(real hardware) 제어용 ROS2 워크스페이스
  src/my_lite6_control/   MoveIt2 기반 시퀀스 제어, grasp pose 계산(matrix_calculator),
                          vision 파이프라인 연동(auto_pick_loop), 그리퍼 제어 등

lite6_sim_ws/      Isaac Sim 시뮬레이션 연동 워크스페이스
  src/my_lite6_bringup/   시뮬레이션 브링업/스폰

assets/            Isaac Sim USD 씬 파일
  f2x_lab_v2.usd
  uf_lite6.usd
```

## 빌드

```bash
cd lite6_ws   # 또는 lite6_sim_ws
colcon build
source install/setup.bash
```

## 주요 실행 노드 (lite6_ws/src/my_lite6_control)

- `sequence_real` — 실제 로봇 pick-and-place 시퀀스
- `sequence_sim` — 시뮬레이션 pick-and-place 시퀀스
- `matrix_calculator_fix` / `matrix_calculator_auto` — 물체 pose → grasp pose 계산
- `auto_pick_loop` — vision 결과 감지 → grasp 계산 → 로봇 실행까지 자동화
