# lite6_motion_control

xArm Lite6 로봇팔 pick-and-place 제어 워크스페이스 (MoveIt2 기반, 실물 로봇 + 시뮬레이션).

- 환경: Ubuntu 24.04, ROS 2 Jazzy
- 테스트한 장비: aarch64 (NVIDIA)

## 구조

```
lite6_ws/                       제어용 ROS 2 워크스페이스 (xarm_ros2 위에 overlay)
  src/my_lite6_control/         MoveIt2 시퀀스 제어, grasp pose 계산, 그리퍼 제어
    config/                     default.yaml(시뮬) / real.yaml(실물) / robots/lite6.yaml
    launch/                     lite6_sim / lite6_real / lite6_custom_ik
    sim/lite6_motion_sim.py     Isaac Sim 모션 뷰어 (선택)
xarm_ros2/                      UFACTORY xarm_ros2 (jazzy, 커밋 4eb468a 고정)
patches/
  xarm_ros2_lite6_custom.patch  xarm_ros2 커스텀 수정 (아래 설치 5단계)
```

`xarm_ros2/`는 커밋 포인터(gitlink)로만 들어 있고, `.gitmodules`는 없습니다. 그래서 clone하면 빈 폴더만 생기며, 설치 4단계에서 직접 받아야 합니다.

`patches/xarm_ros2_lite6_custom.patch`에는 upstream에 없는 아래 파일들이 들어 있습니다. 설치 5단계에서 이 패치를 적용합니다.

- `lite6_moveit_fake_custom.launch.py`, `lite6_moveit_realmove_custom.launch.py` (손목 카메라 포함 URDF 사용)
- `xarm_description/urdf/lite6_complete*.urdf`
- xacro / `xarm_params.yaml` / `moveit_configs_builder.py` 수정

---

## 설치

아래 명령은 한 블록씩 직접 입력하세요. 블록마다 결과를 확인한 뒤 다음 단계로 넘어갑니다.

### 0. 깨끗한 셸에서 시작

`~/.bashrc`가 다른 xarm_ros2 / lite6_ws의 `setup.bash`를 source하고 있으면 underlay가 섞여 빌드가 꼬입니다. 설치와 빌드는 `.bashrc`를 읽지 않는 셸에서 하세요.

먼저 이 줄**만** 단독으로 실행하세요. 여러 줄을 함께 붙여 넣으면 뒤쪽 줄이 사라집니다.

```bash
env -i HOME=$HOME TERM=$TERM PATH=/usr/local/bin:/usr/bin:/bin bash --norc
```

프롬프트가 `bash-5.2$`로 바뀌면 정상입니다. 이어서 아래를 입력합니다.

```bash
export REPO=~/Projects/GDRNPP_bin_picking/lite6_control      # clone 할(한) 위치
export XARM_COMMIT=4eb468a6a2cb778e72d24edff42dfc93ca2bed75
```

conda가 활성화되어 있으면 `conda deactivate`를 먼저 실행하세요. colcon이 conda Python을 잡으면 빌드에 실패합니다.

### 1. 저장소 clone (새 PC에서만)

```bash
git clone https://github.com/f2x-project/lite6_motion_control.git $REPO
cd $REPO
```

### 2. ROS 2 Jazzy 설치 (`/opt/ros/jazzy`가 이미 있으면 건너뜀)

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install -y ros-jazzy-desktop
```

### 3. apt 의존성

```bash
sudo apt install -y git build-essential cmake \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-yaml python3-opencv \
  ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-cv-bridge
```

### 4. xarm_ros2 받기

`xarm_ros2/`가 이미 git 저장소이면(`ls $REPO/xarm_ros2/.git`) 이 단계는 건너뜁니다.

```bash
cd $REPO
rmdir xarm_ros2            # clone 직후 생긴 빈 폴더 제거
git clone -b jazzy https://github.com/xArm-Developer/xarm_ros2.git xarm_ros2
git -C xarm_ros2 checkout $XARM_COMMIT
git -C xarm_ros2 submodule update --init --recursive
```

### 5. 커스텀 패치 적용

```bash
cd $REPO/xarm_ros2
git apply --check ../patches/xarm_ros2_lite6_custom.patch && git apply ../patches/xarm_ros2_lite6_custom.patch
ls xarm_moveit_config/launch | grep custom      # custom 런치 파일 4개가 보이면 OK
```

custom 런치 파일 4개가 이미 보이면 패치가 적용된 상태입니다. 이때 `--check`가 실패하는 것은 정상입니다.

### 6. rosdep

`xarm_gazebo`의 Gazebo Harmonic 키(`sdformat14`, `gz-sim8` 등)는 rosdep 정의가 없어서 skip합니다. 해당 라이브러리는 3단계의 `ros-jazzy-ros-gz`로 설치됩니다.

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init            # "already exists" 에러면 무시
rosdep update
cd $REPO
rosdep install --from-paths xarm_ros2 lite6_ws/src/my_lite6_control --ignore-src --rosdistro jazzy -y \
  --skip-keys "sdformat14 gz-sim8 gz-plugin2 gz-transport13 gz-msgs10 gz-math7 gz-common5"
```

### 7. 빌드

`echo $AMENT_PREFIX_PATH`의 결과는 `/opt/ros/jazzy` 하나만 나와야 합니다. 다른 경로가 섞여 있으면 0단계부터 다시 하세요.

`build/`와 `install/`이 다른 PC에서 만든 것이면 CMake가 경로 불일치로 실패합니다. 예: `CMakeCache.txt ... is different than the directory /home/v/...`. 그래서 처음 빌드할 때는 반드시 지우고 빌드합니다.

**7-1. xarm_ros2** (몇 분 걸립니다)

```bash
cd $REPO/xarm_ros2
rm -rf build install log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**7-2. lite6_ws (xarm_ros2 위에 overlay)**

```bash
source $REPO/xarm_ros2/install/setup.bash
cd $REPO/lite6_ws
rm -rf build install log
colcon build --base-paths src/my_lite6_control
```

`--base-paths`로 패키지 폴더만 지정하는 이유: `lite6_ws/src` 안에 예전 빌드 산출물(`src/build`, `src/install`)이 남아 있어서, 이를 탐색하지 않게 하려는 것입니다.

`SetuptoolsDeprecationWarning`은 무시해도 되는 경고입니다.

### 8. 환경 설정 (터미널마다)

새 터미널에서는 `$REPO`가 비어 있습니다. 비어 있으면 `bash: /xarm_ros2/install/setup.bash: No such file or directory` 에러가 납니다. 그래서 `export REPO`부터 실행합니다.

```bash
export REPO=~/Projects/GDRNPP_bin_picking/lite6_control
source /opt/ros/jazzy/setup.bash
source $REPO/xarm_ros2/install/setup.bash
source $REPO/lite6_ws/install/setup.bash
```

`~/.bashrc`에 넣을 경우, 다른 xarm_ros2 / lite6_ws를 source하는 기존 줄은 지우거나 주석 처리하세요.

---

## 실행

각 명령은 별도 터미널에서 실행하고, 터미널마다 8단계 source를 먼저 해 둡니다.

### 시뮬레이션 (MoveIt fake controller)

```bash
# 터미널 1: MoveIt + fake controller (손목 카메라 URDF)
ros2 launch xarm_moveit_config lite6_moveit_fake_custom.launch.py add_gripper:=true

# 터미널 2: pick-and-place 시퀀스
ros2 run my_lite6_control sequence_sim_action_fix
```

### 실물 로봇

```bash
# 터미널 1: 실물 드라이버 + MoveIt
ros2 launch xarm_moveit_config lite6_moveit_realmove_custom.launch.py robot_ip:=<로봇IP>

# 터미널 2: pick-and-place 시퀀스 (config/real.yaml)
ros2 run my_lite6_control sequence_real_action robot.gripper.service_setup=$REPO/xarm_ros2/install/setup.bash
```

그리퍼 서비스를 호출하기 전에 source하는 파일(`robot.gripper.service_setup`)의 기본값은 `~/xarm_ros2/install/setup.bash`입니다. repo 안의 xarm_ros2를 쓰려면 위처럼 명령줄에서 덮어쓰세요.

### 실물 로봇: 물체 pose → grasp pose → pick

물체의 pose(x y z roll pitch yaw)에서 그리퍼가 잡을 pose를 계산해 실물 로봇으로 집는 순서입니다. 위의 터미널 1(실물 드라이버 + MoveIt)과 터미널 2(`sequence_real_action`)가 먼저 실행 중이어야 합니다.

**1) grasp pose 계산**: [matrix_calculator_fix.py](lite6_ws/src/my_lite6_control/my_lite6_control/matrix_calculator_fix.py)를 실행합니다(터미널 3).

```bash
python3 -m my_lite6_control.matrix_calculator_fix
# > 0.2675 0.1278 0.0243 -45.36 -8.56 -15.20       물체 x y z (m, link_base 기준) roll pitch yaw (deg)
```

출력 예시:

```
obj_x·base_z = 0.287  angle=73.3deg  -> side      ← 물체가 놓인 모양 (upright / side / flipped)
...
0.2638 0.1245 0.0747 134.64 8.56 15.20            ← grasp pose: x y z roll pitch yaw
grasp position = 0.2638 0.1245 0.0747
side: using 'top' rule (obj_z·base_z = 0.695)
```

**2) 로봇에 보내기**: [ee_6d.py](lite6_ws/src/my_lite6_control/my_lite6_control/ee_6d.py)를 실행합니다(터미널 4). 1)에서 나온 `x y z roll pitch yaw` 줄을 그대로 입력합니다.

```bash
python3 -m my_lite6_control.ee_6d
# > 0.2638 0.1245 0.0747 134.64 8.56 15.20
```

`sequence_real_action`은 이 pose를 받아 아래 순서로 움직입니다.

1. pre-grasp: grasp pose에서 툴 z축 방향으로 `sequence.approach_dist_m` 만큼 물러난 위치
2. dive: grasp pose까지 내려감
3. 그리퍼 닫기
4. 들어 올리기
5. `goal` 위치로 이동
6. 그리퍼 열기
7. home으로 복귀

다음 물체는 home으로 돌아온 뒤 보내세요.

**grasp 계산 규칙** ([matrix_calculator_auto.py](lite6_ws/src/my_lite6_control/my_lite6_control/matrix_calculator_auto.py)):

- **자세 분류**: 물체 x축과 위쪽(base z축)의 각도로 나눕니다. 기준값은 `SIDE_THRESHOLD_DEG = 50`입니다.

  | 분류 | 각도 | 그리퍼 방향 |
  |---|---|---|
  | `upright` | 130° 초과 | 물체 축으로 만든 고정 규칙 |
  | `side` | 50° ~ 130° | 물체 z축이 위를 보면 top, 아래를 보면 bottom 규칙 |
  | `flipped` | 50° 미만 | top/down 두 후보를 계산하지만 선택 규칙이 아직 없어서 항상 top을 씀 |

- **잡는 위치**: `물체 위치 + 물체 회전 × 로컬 오프셋`입니다. 오프셋은 파일 위쪽의 `DEFAULT_LOCAL_OFFSET`, `SIDE_LOCAL_OFFSET_*`, `FLIP_LOCAL_OFFSET_*`(m)에 있고, 물체에 맞게 이 값을 조정합니다.
- **같은 로직의 두 파일**: `matrix_calculator_fix`는 대화형이고, `matrix_calculator_auto`는 `compute_grasp()` 함수로 다른 코드에서 불러 씁니다. 두 파일의 계산 로직은 같아서, 오프셋을 바꾸면 **두 파일을 모두** 고쳐야 합니다.

**자동 실행 (`auto_pick_loop`)**: vision 쪽이 `~/received/obj_pose_base_*.json` 또는 `.txt`를 보내면, 이 노드가 grasp를 계산하고 실물 pick까지 이어서 실행합니다. 시퀀스 노드가 내장되어 있으므로 터미널 2의 `sequence_real_action`은 띄우지 않습니다.

```bash
ros2 run my_lite6_control auto_pick_loop
```

- **입력 파일 형식**:
  - `.json`: `[{"xyz_mm": [x, y, z], "rpy_deg": [r, p, y], "model_id": "..."}]`
  - `.txt`: 한 줄에 `x y z roll pitch yaw` (m / deg)
- **이미 있는 파일은 무시**: 시작할 때 폴더에 있던 파일은 처리하지 않고, 새로 들어온 파일만 처리합니다.
- ⚠️ **그리퍼가 동작하지 않습니다**: `auto_pick_loop`는 예전 노드인 `sequence_real`을 사용합니다. 이 노드는 그리퍼를 호출할 때 `/home/v/xarm_ros2/install/setup.bash`를 source하도록 경로가 고정되어 있습니다([sequence_real.py:341](lite6_ws/src/my_lite6_control/my_lite6_control/sequence_real.py#L341)). 이 경로가 없는 PC에서는 그리퍼 호출이 실패하고, 로봇 팔만 움직입니다.

### 목표 pose 보내기

시퀀스 노드는 `/target_pose`(`link_base` 기준 TCP pose)를 받아 움직입니다. 목표가 `safety` 박스(x 0.05~0.45, y -0.45~0.45, z 0.01~0.60 m, reach 0.44 m)를 벗어나면 `outside workspace bounds ... DENIED`를 출력하고 거부합니다.

```bash
python3 -m my_lite6_control.ee_6d          # 입력: x y z | x y z roll pitch yaw | x y z qx qy qz qw
ros2 topic pub --once /target_pose geometry_msgs/msg/Pose "{position: {x: 0.25, y: 0.0, z: 0.15}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}"
```

### 설정 바꾸기

설정 파일 위치는 `lite6_ws/src/my_lite6_control/config/`입니다. 노드가 src의 yaml을 직접 읽기 때문에 yaml 수정만으로는 다시 빌드할 필요가 없습니다.

```bash
ros2 run my_lite6_control moveit2 --print-config                         # 합쳐진 설정 출력
ros2 run my_lite6_control moveit2 planning.planner_id=RRTstar            # 점(.) 경로로 값 덮어쓰기
ros2 run my_lite6_control sequence_sim_action_fix --config ~/my.yaml sequence.approach_dist_m=0.06
```

### 주요 노드

| 노드 | 설명 |
|---|---|
| `sequence_sim_action_fix` | 시뮬레이션 pick-and-place 시퀀스 (`config/default.yaml`) |
| `sequence_real_action` | 실물 pick-and-place 시퀀스 (`config/real.yaml`) |
| `moveit2`, `moveit2_action` | `/target_pose` 하나 → IK → plan → execute → `/ik_done` |
| `auto_pick_loop_sim` | `/tmp/obj_pose_base_*` 감지 → grasp 계산 → 시뮬레이션 실행 (시퀀스 내장) |
| `auto_pick_loop` | `~/received/obj_pose_base_*` 감지 → grasp 계산 → 실물 실행 |
| `view_joint` | 현재 joint 값 출력 |

전체 목록은 [setup.py](lite6_ws/src/my_lite6_control/setup.py)의 `console_scripts`를 보세요.

### Isaac Sim 모션 뷰어 (선택)

[sim/lite6_motion_sim.py](lite6_ws/src/my_lite6_control/sim/lite6_motion_sim.py)는 바닥, 조명, Lite6 로봇만 있는 가벼운 Isaac Sim 장면을 띄웁니다.

- 로봇은 `/joint_states`를 따라 움직이고, 그리퍼는 `/gripper/command`를 따릅니다.
- MoveIt이 실행하는 궤적을 눈으로 보는 뷰어입니다. 물체를 집는 물리 시뮬레이션은 하지 않습니다.

Isaac Sim과 Isaac Lab은 별도로 설치되어 있어야 합니다(이 PC 기준 `~/IsaacSim`, `~/IsaacLab`). 경로는 `config/default.yaml`의 `sim:` 섹션(`sim.experience`, `sim.robot_usd`)에서 설정합니다.

각 터미널에서 8단계 source를 먼저 합니다.

```bash
# 터미널 1: MoveIt + fake controller
ros2 launch xarm_moveit_config lite6_moveit_fake_custom.launch.py add_gripper:=true

# 터미널 2: Isaac Sim 뷰어
~/IsaacLab/isaaclab.sh -p $REPO/lite6_ws/src/my_lite6_control/sim/lite6_motion_sim.py

# 터미널 3: pick-and-place 시퀀스
ros2 run my_lite6_control sequence_sim_action_fix
```

- **ROS source가 필요합니다**: 스크립트가 `rclpy`를 직접 import합니다. Isaac Sim의 Python이 3.12라서 Jazzy와 버전이 맞습니다.
- **`isapython`/`isim` alias는 쓰지 마세요**: 이런 alias는 `PYTHONPATH`를 지우기 때문에 `rclpy`를 찾지 못합니다.
- **스크립트 주석의 `lab ...`**: `isaaclab.sh -p`를 줄여 쓴 alias입니다.
- **옵션 예시**:
  - `... lite6_motion_sim.py --headless`
  - `... lite6_motion_sim.py sim.initial_pose=init`
  - `... lite6_motion_sim.py sim.render.mode=RealTimePathTracing`
- **예전 브링업 코드**: 빈 맵과 카메라가 포함된 예전 브링업(`lite6_sim_ws/src/my_lite6_bringup`)은 커밋 `efabf2e`에서 삭제되었습니다. 필요하면 `git show efabf2e^:<경로>`로 꺼낼 수 있습니다.

---

## 문제 해결

| 증상 | 해결 |
|---|---|
| `CMakeCache.txt ... is different than the directory ...` | 다른 PC에서 만든 빌드 결과물이 원인입니다. `rm -rf build install log` 후 다시 빌드하세요. |
| `Some selected packages are already built in one or more underlay workspaces` | 다른 xarm_ros2가 source된 상태입니다. 0단계의 깨끗한 셸에서 빌드하세요. |
| `Cannot locate rosdep definition for [sdformat14]` 등 | 6단계의 `--skip-keys`를 붙이세요. |
| `bash: /xarm_ros2/install/setup.bash: No such file or directory` | 새 터미널이라 `$REPO`가 비어 있습니다. `export REPO=~/Projects/GDRNPP_bin_picking/lite6_control`을 먼저 실행하세요. |
| 붙여 넣은 명령 중 일부가 실행되지 않음 | `env -i ... bash --norc` 줄은 단독으로 실행하세요. |
| `file 'lite6_moveit_fake_custom.launch.py' was not found in the share directory ... /home/<user>/xarm_ros2/install/...` | `~/.bashrc`가 다른 xarm_ros2를 source하고 있어서 생기는 문제입니다. `ros2 pkg prefix xarm_moveit_config`의 결과가 `$REPO/xarm_ros2/install`이어야 합니다. 8단계 source를 다시 하거나, `~/.bashrc`의 source 줄을 repo 경로로 바꾸세요. |
| `$REPO/xarm_ros2/install`에서도 `lite6_moveit_*_custom.launch.py`를 찾을 수 없음 | 5단계 패치를 적용한 뒤 7-1단계를 다시 빌드하세요. |
