# 미지 환경 온라인 SLAM 실행 및 품질 가이드

## 현재 프로파일

- D435 RGB/aligned depth: `848x480x30`, High Accuracy preset
- WT901C485 `0x50`/`0x51`/`0x52`: 각각 약 45 Hz 목표
- RGB-D visual odometry: 카메라 입력 속도에 근접
- EKF: 45 Hz
- RTAB-Map 지도/그래프 갱신: 융합/카메라 전용 2 Hz
- 3D occupancy cell: 2 cm, depth decimation 1
- 임시 TF: `camera_link -> imu_50_link = (-0.05, 0, +0.02) m`, 회전 0

30 Hz는 센서와 odometry 목표 주기다. 누적 3D 지도 생성, 특징 비교와
loop closure를 매 프레임 수행하면 CPU, DB와 네트워크 사용량만 크게
늘 수 있으므로 별도 주기로 운용한다.

## D435 근거리 품질 프로파일

현재 설정은 얇고 가까운 장애물의 형상을 보존하도록 다음처럼 구성한다.

- `848x480x30`, High Accuracy, emitter/laser 활성화
- disparity 기반 spatial filter 활성화
- 진동·이동 중 과거 깊이 잔상을 피하기 위해 temporal filter 비활성화
- RTAB-Map 유효 깊이 `0.18~3.5 m`, 2 cm cell, depth decimation 1
- 4 cm 반경에서 이웃이 2개 미만인 고립점 제거

D435의 848x480 최소 깊이는 약 19.5 cm이므로 실제 운용에서는 카메라 앞
20 cm 이상을 신뢰 구간으로 본다. 이 거리보다 가까운 깊이가 비거나 튀는 것은
SLAM 파라미터로 복원할 수 없다. 10~20 cm 충돌 감지가 필수라면 짧은 거리용
ToF/IR/범퍼를 안전 계층에 추가하거나 D405 같은 근거리 카메라를 검토한다.

High Accuracy는 잘못된 점을 줄이는 대신 깊이 구멍이 늘 수 있다. 실제 현장에서
벽이 깨끗하지만 빈 영역이 지나치게 많다면 `depth_module.visual_preset`을 Medium
Density 계열로 A/B 비교한다. 재질, 조명, 반사와 진동 조건을 고정하고 같은 경로를
두 번 기록해 이중 벽 두께와 VO lost 횟수로 판단한다.

## 1. 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select wt901c485_driver my_robot_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 2. 실제 운용 형태: D435 + IMU + EKF

새 현장에서는 새 DB 경로와 `reset_database:=true`를 사용한다. 이 옵션은
지정 DB를 삭제하므로 경로를 반드시 먼저 확인한다.

```bash
ros2 launch my_robot_bringup rtabmap_mapping.launch.py \
  database_path:=~/.ros/mission_001.db \
  reset_database:=true \
  rviz:=true \
  rtabmap_viz:=false
```

RViz에는 탐색한 곳만 `Explored 3D Map`으로 나타나고 현재 로봇 화살표,
초록색 실제 궤적, TF와 RGB 영상이 함께 표시된다. RTAB-Map의 별도 분석
GUI가 필요할 때만 `rtabmap_viz:=true`로 바꾼다.

게임 지도처럼 로봇을 중심으로 위에서 보려면 `topdown:=true`를 추가한다.

```bash
ros2 launch my_robot_bringup rtabmap_mapping.launch.py \
  database_path:=~/.ros/mission_001.db \
  reset_database:=false \
  rviz:=true topdown:=true
```

녹색 화살표는 출발점, 주황색 화살표는 현재 위치, 녹색 선은 실제 동선이다.
로봇 위의 텍스트는 RGB, depth, VO, SLAM, IMU와 EKF 수신 Hz를 표시한다.

## 3. 임시 검증 형태: 카메라만 사용

IMU 노드와 EKF를 시작하지 않고 D435 특징점만으로 온라인 SLAM을 한다.

```bash
ros2 launch my_robot_bringup rtabmap_camera_only_mapping.launch.py \
  database_path:=~/.ros/mission_camera_only.db \
  reset_database:=true \
  rviz:=true \
  rtabmap_viz:=false
```

이 모드에서는 RGB-D odometry가 유일한 `odom -> base_link` 발행자다.
빠른 회전, 머리 진동, motion blur 또는 무늬 없는 벽에서 특징을 잃을 때
IMU가 보완하지 못하므로 실기 최종 형태로 간주하지 않는다.

## 4. 실행 중 확인

별도 터미널에서 다음을 확인한다.

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /visual_odom
ros2 topic hz /imu_50/data
ros2 topic hz /imu_51/data
ros2 topic hz /imu_52/data
ros2 topic hz /odometry/filtered

ros2 topic echo /slam/current_pose --once
ros2 topic echo /slam/distance_traveled
ros2 topic echo /slam/distance_from_start
ros2 topic echo /slam/rate_summary
ros2 topic echo /slam/diagnostics
ros2 run tf2_ros tf2_echo map base_link
```

카메라 전용 모드에는 `/imu_50/data`~`/imu_52/data`와 `/odometry/filtered`가 없는 것이
정상이다. 누적 지도 입력은 `/mapData`, 2D 점유 지도는 구독자가 있을 때
`/map`으로 발행된다.

RViz에서 지도가 보이지 않으면 다음 순서로 확인한다.

```bash
ros2 topic info /mapData -v
ros2 topic hz /mapData
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

`/mapData`가 약 2 Hz인 것은 설정대로 정상이다. 카메라 토픽까지 그 속도로
떨어졌다는 뜻은 아니다.

## 5. 주행 방법

1. 시작 직후 IMU 초기화 동안 약 2초간 정지한다.
2. 0.1~0.2 m/s에서 시작하고 고속 제자리 회전을 피한다.
3. 코너나 큰 자세 변화 전후에 잠시 속도를 줄인다.
4. 출발점과 교차점을 다시 지나 loop closure를 만든다.
5. D435 바로 앞의 근거리 사각과 머리 흔들림을 고려해 별도 근접 센서나
   범퍼를 안전 계층에 둔다.
6. 케이블과 센서가 서로 움직이지 않게 단단히 고정하고 노출 부족으로
   blur가 생기지 않게 조명을 확보한다.

좁은 통로에서 D435가 모든 근거리 장애물을 보장하지는 않는다. 지도용
depth와 충돌 방지는 다른 안전 기능이며, 최종 로봇에는 측면/근거리 ToF,
IR 또는 접촉 센서를 함께 검토한다.

## 6. 원시 데이터도 함께 기록

현장 실패를 재현할 수 있도록 여유 저장공간이 있을 때 rosbag을 남긴다.

```bash
ros2 bag record \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw \
  /imu_50/data /imu_51/data /imu_52/data /visual_odom /odometry/filtered \
  /slam/current_pose /slam/path \
  /tf /tf_static
```

bag과 `.db`는 Git에 커밋하지 않는다.

## 7. 정상 종료와 DB 확인

실행 터미널에서 `Ctrl+C`를 한 번 누르고 RTAB-Map의 database saved/closed
로그가 끝난 뒤 터미널을 닫는다.

```bash
rtabmap-databaseViewer ~/.ros/mission_001.db
```

현재 상태를 파일 크기와 수정 시각으로도 확인할 수 있다.

```bash
ls -lh ~/.ros/mission_001.db
```

## 8. 초기 합격 기준

- 10분 주행 중 반복적인 VO lost/reset이 없음
- 시작점 재방문 시 loop closure가 수락됨
- 출발점 복귀 오차 0.20 m와 5도 이내
- 벽이 이중으로 겹치거나 바닥이 심하게 물결치지 않음
- 현재 pose와 궤적이 3D 지도 위에서 연속적으로 움직임
- `/mapData` 처리 주기가 장시간 떨어지거나 queue가 계속 늘지 않음
- `.db`가 정상 종료 후 다시 열림

문제가 있으면 EKF 수치를 먼저 임의 조절하지 말고 입력 주기/USB 오류,
노출과 blur, 고정 TF, IMU 축·timestamp·covariance, VO inlier, loop closure
순서로 원인을 분리한다.

## 9. 후속 앱 스트리밍

RViz 검증이 끝나기 전에는 고해상도 전체 cloud를 앱으로 보내지 않는다.
먼저 `/mapData`와 생성된 cloud의 실제 크기, 지연, CPU/GPU와 네트워크를
측정한다. 이후 map streamer가 5~10 cm voxel, 공간 tile, 변경분 전송을
적용해 앱 표시용 데이터를 별도로 만든다.

앱 전송 해상도는 SLAM 원본 품질과 분리해 조절한다. RViz와 RTAB-Map DB를
정확도 기준으로 유지하고, gateway의 voxel 크기와 최대 점 개수는 네트워크와
브라우저 성능에 맞춰 낮춘다.
