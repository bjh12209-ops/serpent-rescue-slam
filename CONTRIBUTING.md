# 협업 및 GitHub 작업 규칙

## 저장소 범위

이 저장소는 D435와 WT901C485를 이용한 ROS 2 SLAM, EKF, RTAB-Map,
텔레메트리 gateway와 브라우저 UI만 관리한다. 모터, Dynamixel, 조이스틱,
구동 정책과 액추에이터 제어 코드는 별도 저장소에서 관리한다.

## 기본 작업 흐름

`main`에서 직접 개발하지 않고 기능별 브랜치와 Pull Request를 사용한다.

```bash
cd ~/ros2_ws
git switch main
git pull --ff-only origin main
git switch -c feature/slam-quality

# 수정 및 검증
git status --short
git diff --check

git add -- <검토한 파일>
git commit -m "feat: describe the change"
git push -u origin feature/slam-quality
```

권장 브랜치 이름:

```text
feature/sensor-calibration
feature/slam-quality
feature/person-detection
feature/web-telemetry
fix/camera-latency
docs/setup-guide
```

GitHub에서 Draft Pull Request를 열고 최소 한 명이 검토한 뒤 병합한다.
PR에는 변경 목적, 하드웨어, 실행 명령, 테스트 결과와 알려진 제한을 적는다.

## 담당 영역

- `src/wt901c485_driver`: IMU/RS485 담당
- `src/my_robot_bringup`: D435, VO, EKF, RTAB-Map 및 텔레메트리 담당
- `person_pose_detector.py`: 카메라 사람 감지 담당
- `apps/mission_control_ui`: 웹 UI 담당

## Git에 올리지 않는 파일

- `build/`, `install/`, `log/`
- RTAB-Map `.db`, rosbag, point cloud와 mesh
- YOLO `.pt`, ONNX, TensorRT 엔진
- `.env`, SSH 개인 키, 토큰과 장치별 비밀정보

대용량 지도와 모델은 팀의 별도 데이터 저장소나 GitHub Release를 사용하고
README에 버전과 checksum을 기록한다.

## PR 전 검증

```bash
source /opt/ros/humble/setup.bash
rosdep check --from-paths src --ignore-src --rosdistro humble
colcon build --symlink-install \
  --packages-select wt901c485_driver my_robot_bringup
source install/setup.bash
colcon test --packages-select wt901c485_driver my_robot_bringup
colcon test-result --verbose
git diff --check
git status --short --branch
```

실물 센서 시험 시 OS/ROS 버전, 카메라와 IMU 식별 정보, 토픽 주기,
시험 시간, 이동 경로, 누락 프레임과 지도 화면을 PR에 기록한다.
