# Jetson Orin Nano Super 배포 가이드

이 문서는 **Jetson Orin Nano Super + JetPack 6 계열(Ubuntu 22.04) + ROS 2
Humble**을 기준으로 한다. 구형 `Jetson Nano`(Maxwell, JetPack 4)는 Ubuntu
22.04/ROS 2 Humble 바이너리 환경이 아니므로 이 저장소를 그대로 실행하는
대상이 아니다.

## 1. 저장소만 clone하면 끝나는가?

아니다. 공개 GitHub 저장소에는 소스와 설정만 있으며 다음 항목은 장치에서
별도로 준비해야 한다.

- JetPack/Ubuntu와 ROS 2 Humble arm64
- D435용 librealsense 및 `realsense2_camera`
- RTAB-Map, robot_localization 등 ROS 의존성
- USB-RS485 권한과 WT901 장치 경로
- JetPack 버전에 맞는 NVIDIA PyTorch, Ultralytics와 YOLO 모델
- RTAB-Map DB와 TensorRT engine 같은 장치별 생성 파일

DB, rosbag, `*.pt`, `*.engine`, `build/`, `install/`은 의도적으로 Git에
포함하지 않는다.

## 2. 기본 OS와 ROS 준비

먼저 JetPack 6 계열로 플래시한 뒤 다음 값이 맞는지 확인한다.

```bash
uname -m
lsb_release -a
cat /etc/nv_tegra_release
```

예상 아키텍처는 `aarch64`, OS는 Ubuntu 22.04다. ROS 공식 Ubuntu deb 설치
절차로 ROS 2 Humble을 설치한 후 개발 도구를 준비한다.

```bash
sudo apt update
sudo apt install -y git python3-rosdep python3-colcon-common-extensions \
  python3-opencv python3-websockets python3-pip python3-venv \
  ros-humble-robot-localization ros-humble-rtabmap-ros \
  ros-humble-realsense2-camera

sudo rosdep init 2>/dev/null || true
rosdep update
```

`realsense-viewer` 또는 ROS 카메라 노드에서 D435가 보이지 않으면 임의의
커널 패치부터 적용하지 말고, librealsense의 Jetson 공식 설치 문서에서 현재
JetPack/L4T 버전과 호환되는 Debian, RSUSB, native backend 중 하나를 선택한다.

## 3. 공개 저장소 clone과 빌드

```bash
git clone https://github.com/bjh12209-ops/serpent-rescue-slam.git ~/ros2_ws
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --parallel-workers 2 \
  --packages-select wt901c485_driver my_robot_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

빌드가 메모리 부족으로 종료되면 다른 앱을 닫고 `--parallel-workers 1`로
다시 빌드한다.

## 4. 센서부터 단계별 확인

D435는 USB 3 포트에 직접 연결하고, IMU 어댑터 경로와 권한을 확인한다.

```bash
rs-enumerate-devices -s
ls -l /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_bringup sensor_bringup.launch.py
```

다른 터미널에서 다음 주기를 확인한다.

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /imu_50/data
ros2 topic hz /imu_51/data
ros2 topic hz /imu_52/data
```

## 5. YOLO 없이 SLAM과 웹 UI 먼저 검증

Jetson에서는 RViz와 RTAB-Map GUI를 끄고 웹 UI를 다른 PC에서 보는 편이
가볍다.

```bash
# 터미널 1
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup rtabmap_mapping.launch.py \
  database_path:=~/.ros/mission_001.db reset_database:=true \
  rviz:=false rtabmap_viz:=false
```

```bash
# 터미널 2
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup mission_control.launch.py \
  enable_person_detection:=false \
  ui_host:=0.0.0.0 websocket_host:=0.0.0.0 \
  max_cloud_points:=20000 camera_stream_rate:=15.0
```

같은 신뢰 가능한 LAN의 PC 브라우저에서
`http://<JETSON_IP>:8080`을 연다. 이 포트에는 인증과 TLS가 없으므로 공용
인터넷에 노출하지 않는다.

## 6. Jetson GPU용 YOLO 추가

x86 CPU용 PyTorch wheel이나 x86에서 생성한 TensorRT engine을 복사하면 안
된다. 현재 JetPack 버전과 맞는 NVIDIA PyTorch를 Jetson에 설치한 뒤
Ultralytics를 설치한다.

```bash
cd ~/ros2_ws
python3 -m venv --system-site-packages .venv-yolo
source .venv-yolo/bin/activate

# 이 단계 전에 NVIDIA의 현재 JetPack-PyTorch 호환표에서 맞는 wheel/container를
# 선택해 torch와 torchvision을 설치한다.
python3 -m pip install 'numpy<2' ultralytics
python3 -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
```

모델은 Jetson에서 준비하고 먼저 CUDA 추론으로 실행한다.

```bash
ros2 launch my_robot_bringup mission_control.launch.py \
  yolo_model:=$HOME/ros2_ws/yolo26n-pose.pt \
  yolo_device:=0 yolo_image_size:=320 yolo_rate:=2.0 \
  ui_host:=0.0.0.0 websocket_host:=0.0.0.0 \
  max_cloud_points:=20000 camera_stream_rate:=15.0
```

기능 확인 후 해당 Jetson에서 TensorRT engine을 생성해 `yolo_model`로
지정한다. engine은 JetPack/TensorRT/GPU 조합에 종속되므로 Git에 올리지
않는다.

## 7. 재부팅 후 실행 원칙

- 새 지도를 만들 때만 `reset_database:=true`를 사용한다.
- 이어서 매핑할 때는 반드시 `reset_database:=false`를 사용한다.
- DB를 초기화한 뒤 웹 게이트웨이를 재시작하면 점군 캐시와 사람 표식도 새
  세션으로 시작한다.
- 최종 센서 장착 후 `camera_link -> imu_50_link` TF를 다시 실측한다.
- SBC에서 검증이 끝나기 전에는 자동 부팅 서비스나 모터 제어를 연결하지 않는다.
