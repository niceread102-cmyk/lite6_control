#!/usr/bin/env bash
# lite6_motion_control 설치 스크립트 (Ubuntu 24.04 + ROS 2 Jazzy)
#
#   1. ROS 2 Jazzy (없으면 설치)
#   2. apt 의존성 (MoveIt2, ros2_control, Gazebo 브리지, Python 패키지)
#   3. xarm_ros2 clone (고정 커밋) + 서브모듈 + patches/xarm_ros2_lite6_custom.patch 적용
#   4. rosdep 의존성
#   5. xarm_ros2 빌드 -> lite6_ws 빌드 (xarm_ros2 위에 overlay)
#
# 사용법: ./install.sh [옵션]   (./install.sh --help)
set -eo pipefail

ROS_DISTRO_NAME=jazzy
XARM_REPO=https://github.com/xArm-Developer/xarm_ros2.git
XARM_BRANCH=jazzy
XARM_COMMIT=4eb468a6a2cb778e72d24edff42dfc93ca2bed75   # 상위 repo의 xarm_ros2 gitlink와 같은 커밋

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XARM_DIR="$REPO_ROOT/xarm_ros2"
LITE6_WS="$REPO_ROOT/lite6_ws"
XARM_PATCH="$REPO_ROOT/patches/xarm_ros2_lite6_custom.patch"

# xarm_gazebo의 Gazebo Harmonic 키: rosdep 정의가 없고 ros-jazzy-ros-gz(vendor 패키지)로 대신 설치됨
ROSDEP_SKIP_KEYS="sdformat14 gz-sim8 gz-plugin2 gz-transport13 gz-msgs10 gz-math7 gz-common5"

APT_PACKAGES=(
    git curl build-essential cmake
    python3-colcon-common-extensions python3-rosdep python3-pip
    python3-numpy python3-scipy python3-yaml python3-opencv
    ros-$ROS_DISTRO_NAME-moveit
    ros-$ROS_DISTRO_NAME-ros2-control
    ros-$ROS_DISTRO_NAME-ros2-controllers
    ros-$ROS_DISTRO_NAME-ros-gz
    ros-$ROS_DISTRO_NAME-gz-ros2-control
    ros-$ROS_DISTRO_NAME-cv-bridge
)

SKIP_ROS=0
SKIP_APT=0
SKIP_XARM_BUILD=0
CLEAN=0
ADD_BASHRC=0
JOBS="$(nproc)"

usage() {
    cat <<EOF
사용법: $0 [옵션]

  --skip-ros          ROS 2 Jazzy 설치 단계를 건너뜀 (/opt/ros/$ROS_DISTRO_NAME 가 이미 있어야 함)
  --skip-apt          apt / rosdep 의존성 설치를 건너뜀 (sudo 없이 빌드만)
  --skip-xarm-build   xarm_ros2 빌드를 건너뜀 (이미 빌드된 경우 lite6_ws 만 다시 빌드)
  --clean             빌드 전에 build/ install/ log/ 를 지우고 처음부터 빌드
  --bashrc            ~/.bashrc 에 setup.bash source 줄을 추가
  -j, --jobs N        병렬 빌드 작업 수 (기본: $JOBS)
  -h, --help          이 도움말
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-ros)        SKIP_ROS=1 ;;
        --skip-apt)        SKIP_APT=1 ;;
        --skip-xarm-build) SKIP_XARM_BUILD=1 ;;
        --clean)           CLEAN=1 ;;
        --bashrc)          ADD_BASHRC=1 ;;
        -j|--jobs)         JOBS="$2"; shift ;;
        -h|--help)         usage; exit 0 ;;
        *) echo "알 수 없는 옵션: $1" >&2; usage; exit 1 ;;
    esac
    shift
done

step() { echo; echo -e "\033[1;34m==> $*\033[0m"; }
warn() { echo -e "\033[1;33m[경고]\033[0m $*" >&2; }
die()  { echo -e "\033[1;31m[오류]\033[0m $*" >&2; exit 1; }

# ---------------------------------------------------------------------------
step "0/5 환경 확인"
. /etc/os-release
echo "OS: $PRETTY_NAME ($(uname -m)), repo: $REPO_ROOT"
if [[ "${VERSION_ID:-}" != "24.04" ]]; then
    warn "ROS 2 Jazzy는 Ubuntu 24.04 기준입니다 (현재 ${VERSION_ID:-unknown}). 계속 진행합니다."
fi
[[ $EUID -eq 0 ]] && die "root 로 실행하지 마세요. 필요한 단계에서 sudo 를 사용합니다."
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    warn "conda 환경($CONDA_PREFIX)이 활성화되어 있습니다. colcon 빌드가 시스템 Python 대신 conda Python 을 쓰면 실패하므로 'conda deactivate' 후 실행하는 것을 권장합니다."
fi

# ---------------------------------------------------------------------------
step "1/5 ROS 2 $ROS_DISTRO_NAME"
if [[ -f /opt/ros/$ROS_DISTRO_NAME/setup.bash ]]; then
    echo "이미 설치됨: /opt/ros/$ROS_DISTRO_NAME"
elif [[ $SKIP_ROS -eq 1 || $SKIP_APT -eq 1 ]]; then
    die "/opt/ros/$ROS_DISTRO_NAME 가 없습니다. --skip-ros / --skip-apt 없이 다시 실행하세요."
else
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl locales
    sudo add-apt-repository -y universe
    sudo locale-gen en_US en_US.UTF-8
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    [[ -n "$ROS_APT_SOURCE_VERSION" ]] || die "ros-apt-source 최신 버전을 가져오지 못했습니다 (네트워크 확인)."
    tmp_deb="$(mktemp --suffix=.deb)"
    curl -fL -o "$tmp_deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME:-$VERSION_CODENAME}_all.deb"
    sudo dpkg -i "$tmp_deb"
    rm -f "$tmp_deb"
    sudo apt-get update
    sudo apt-get install -y ros-$ROS_DISTRO_NAME-desktop
fi

# ---------------------------------------------------------------------------
step "2/5 apt 의존성"
if [[ $SKIP_APT -eq 1 ]]; then
    echo "건너뜀 (--skip-apt)"
else
    sudo apt-get update
    sudo apt-get install -y "${APT_PACKAGES[@]}"
fi

# ---------------------------------------------------------------------------
step "3/5 xarm_ros2 소스 준비"
if [[ ! -d "$XARM_DIR/.git" && ! -f "$XARM_DIR/.git" ]]; then
    # 상위 repo를 clone 하면 xarm_ros2 는 빈 디렉터리(gitlink)로만 남는다
    if [[ -d "$XARM_DIR" ]] && [[ -n "$(ls -A "$XARM_DIR")" ]]; then
        die "$XARM_DIR 가 비어 있지 않은데 git 저장소가 아닙니다. 확인 후 지우고 다시 실행하세요."
    fi
    rmdir "$XARM_DIR" 2>/dev/null || true
    git clone -b "$XARM_BRANCH" "$XARM_REPO" "$XARM_DIR"
    git -C "$XARM_DIR" checkout -q "$XARM_COMMIT"
else
    head="$(git -C "$XARM_DIR" rev-parse HEAD)"
    if [[ "$head" != "$XARM_COMMIT" ]]; then
        warn "xarm_ros2 HEAD($head)가 고정 커밋($XARM_COMMIT)과 다릅니다. 그대로 진행합니다."
    else
        echo "xarm_ros2 이미 있음 (커밋 ${XARM_COMMIT:0:7})"
    fi
fi
git -C "$XARM_DIR" submodule update --init --recursive

# 커스텀 런치 파일(lite6_moveit_*_custom.launch.py), 손목 카메라 URDF, xacro 수정
if git -C "$XARM_DIR" apply --reverse --check "$XARM_PATCH" 2>/dev/null; then
    echo "커스텀 패치 이미 적용됨"
elif git -C "$XARM_DIR" apply --check "$XARM_PATCH" 2>/dev/null; then
    git -C "$XARM_DIR" apply --whitespace=nowarn "$XARM_PATCH"
    echo "커스텀 패치 적용 완료: $(basename "$XARM_PATCH")"
else
    die "패치를 적용할 수 없습니다: $XARM_PATCH
xarm_ros2 에 다른 로컬 수정이 있는지 'git -C $XARM_DIR status' 로 확인하세요."
fi

# ---------------------------------------------------------------------------
step "4/5 rosdep 의존성"
source /opt/ros/$ROS_DISTRO_NAME/setup.bash
if [[ $SKIP_APT -eq 1 ]]; then
    echo "건너뜀 (--skip-apt)"
else
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        sudo rosdep init
    fi
    rosdep update --rosdistro "$ROS_DISTRO_NAME"
    rosdep install --from-paths "$XARM_DIR" "$LITE6_WS/src/my_lite6_control" \
        --ignore-src --rosdistro "$ROS_DISTRO_NAME" -y --skip-keys "$ROSDEP_SKIP_KEYS"
fi

# ---------------------------------------------------------------------------
step "5/5 빌드"
if [[ $CLEAN -eq 1 ]]; then
    echo "build/ install/ log/ 삭제"
    rm -rf "$LITE6_WS"/{build,install,log}
    [[ $SKIP_XARM_BUILD -eq 1 ]] || rm -rf "$XARM_DIR"/{build,install,log}
fi

if [[ $SKIP_XARM_BUILD -eq 1 ]]; then
    echo "xarm_ros2 빌드 건너뜀 (--skip-xarm-build)"
    [[ -f "$XARM_DIR/install/setup.bash" ]] || die "$XARM_DIR/install/setup.bash 가 없습니다. --skip-xarm-build 없이 실행하세요."
else
    echo "-- xarm_ros2"
    (cd "$XARM_DIR" && colcon build --parallel-workers "$JOBS" --cmake-args -DCMAKE_BUILD_TYPE=Release)
fi

echo "-- lite6_ws (overlay)"
source "$XARM_DIR/install/setup.bash"
(cd "$LITE6_WS" && colcon build --base-paths src/my_lite6_control --parallel-workers "$JOBS")

# ---------------------------------------------------------------------------
SOURCE_LINES="source /opt/ros/$ROS_DISTRO_NAME/setup.bash
source $XARM_DIR/install/setup.bash
source $LITE6_WS/install/setup.bash"

if [[ $ADD_BASHRC -eq 1 ]]; then
    if grep -qF "$LITE6_WS/install/setup.bash" ~/.bashrc 2>/dev/null; then
        echo "~/.bashrc 에 이미 추가되어 있음"
    else
        printf '\n# lite6_motion_control\n%s\n' "$SOURCE_LINES" >> ~/.bashrc
        echo "~/.bashrc 에 source 줄 추가"
    fi
fi

step "설치 완료"
cat <<EOF
새 터미널마다 아래를 실행하세요 (또는 ./install.sh --bashrc 로 ~/.bashrc 에 등록):

$SOURCE_LINES

실행 방법은 README.md 를 참고하세요.
EOF
