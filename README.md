# SERPENT Rescue SLAM

Ubuntu 22.04와 ROS 2 Humble에서 Intel RealSense D435, WT901C485 IMU,
`robot_localization` EKF와 RTAB-Map을 이용해 온라인 RGB-D SLAM을 수행하고
브라우저 관제 UI로 전송하는 워크스페이스입니다.

이 저장소의 범위는 센서 기반 위치 추정, 지도 생성, 사람 감지 결과 표시와
읽기 전용 웹 텔레메트리까지입니다. 모터, Dynamixel, 조이스틱 및 로봇
구동 제어 코드는 포함하지 않습니다.

## 구성

```text
D435 RGB + aligned depth -> RGB-D visual odometry --┐
WT901C485 IMU -------------------------------> EKF --┤
                                                     v
                                            RTAB-Map SLAM
                                                     |
                          2D map / 3D cloud / pose / path / rates
                                                     |
                                      ROS 2 telemetry gateway
                                                     |
                                        Browser Mission Control UI
```

- `src/wt901c485_driver`: USB-RS485 WT901C485 ROS 2 드라이버
- `src/my_robot_bringup`: D435, EKF, RTAB-Map, RViz, 텔레메트리 및 YOLO 실행
- `apps/mission_control_ui`: 2D/3D 지도, 경로, 위치, 카메라와 상태 웹 화면
- `docs`: 시스템 구조, 지도 품질과 UI 설계 문서

## 현재 검증 하드웨어

- Ubuntu 22.04, ROS 2 Humble
- Intel RealSense D435
- WitMotion WT901C485 한 개, Modbus 주소 `0x52`
- USB-RS485 어댑터
- 임시 센서 TF: 카메라는 IMU보다 x축 전방으로 `0.05 m`

최종 장착 후에는 카메라와 IMU의 실제 위치와 회전을 다시 측정해 고정 TF를
수정해야 합니다.

## 새 PC에서 내려받기

저장소 Collaborator로 초대받고 SSH 키를 GitHub에 등록한 뒤 실행합니다.

```bash
git clone git@github.com:bjh12209-ops/serpent-rescue-slam.git ~/ros2_ws
cd ~/ros2_ws
```

필수 ROS 패키지를 설치합니다.

```bash
sudo apt update
sudo apt install -y \
  ros-humble-realsense2-camera \
  ros-humble-robot-localization \
  ros-humble-rtabmap-ros \
  python3-opencv \
  python3-websockets \
  python3-rosdep
```

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install \
  --packages-select wt901c485_driver my_robot_bringup
source install/setup.bash
```

## 온라인 SLAM 실행

새 테스트 지도를 만들 때:

```bash
ros2 launch my_robot_bringup rtabmap_mapping.launch.py \
  database_path:=~/.ros/mission_001.db \
  reset_database:=true \
  rviz:=true rtabmap_viz:=false
```

같은 DB를 이어서 사용할 때는 `reset_database:=false`로 변경합니다.
`reset_database:=true`는 지정한 DB를 초기화하므로 경로를 반드시
확인하세요.

## 웹 관제 UI 실행

SLAM을 실행한 상태에서 새 터미널을 엽니다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup mission_control.launch.py
```

브라우저에서 `http://localhost:8080`을 엽니다. 2D 지도, 3D 점군,
현재 위치와 방향, 실제 동선, 주행거리, ROS 토픽 주기와 D435 RGB 영상을
확인할 수 있습니다.

자세한 실행과 진단은
[SLAM 패키지 가이드](src/my_robot_bringup/README.md),
[지도 품질 가이드](docs/mapping_quality.md),
[웹 UI 가이드](apps/mission_control_ui/README.md)를 참고하세요.

## Git에 포함하지 않는 데이터

RTAB-Map DB, rosbag, point cloud/mesh, YOLO 모델, `build/`, `install/`,
`log/`, 가상환경과 인증정보는 저장소에 커밋하지 않습니다.
