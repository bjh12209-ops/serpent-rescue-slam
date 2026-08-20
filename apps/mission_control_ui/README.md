# Mission Control UI prototype

재난구조 뱀 로봇의 PC 브라우저 관제 화면 시제품이다. 별도 패키지 설치 없이
mock telemetry로 동작하며, ROS 2에 직접 접속하지 않는다.

## 실행

```bash
cd ~/ros2_ws
python3 -m http.server 8080 --directory apps/mission_control_ui
```

브라우저에서 `http://localhost:8080`을 연다. 지도 드래그, 휠 확대/축소,
로봇 추적, 2D/3D 전환을 사용할 수 있다.

## 실제 ROS 2 연결

먼저 한 터미널에서 SLAM을 실행한 뒤, 다른 터미널에서 gateway를 실행한다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup mission_control.launch.py
```

기본 PC 고해상도 프로필은 2D 셀 5 cm, 유효 3D 점 최대 50,000개,
높이 -1.5~3.0 m를 전송한다. 성능이 부족한 SBC에서는 실행 인자로 낮출 수 있다.

```bash
ros2 launch my_robot_bringup mission_control.launch.py \
  map_cell_size:=0.08 max_cloud_points:=20000
```

이제 `http://localhost:8080`을 열면 gateway 주소가 자동으로 붙고 실제
ROS 토픽을 표시한다. 별도의 `python3 -m http.server`는 실행하지 않는다.

전방 카메라는 텔레메트리 WebSocket과 분리된 `/camera.mjpg`에서 최신 JPEG만
20 FPS로 전달한다. 대용량 3D 점군이 전송 중이어도 오래된 카메라 프레임이
WebSocket 큐에 쌓이지 않는다. PC 여유가 있으면 30 FPS로 올릴 수 있다.

```bash
ros2 launch my_robot_bringup mission_control.launch.py \
  camera_publish_rate:=30.0 camera_stream_rate:=30.0
```

gateway는 읽기 전용이다. 브라우저 메시지 중 연결 지연 측정용 `ping`만
처리하며 로봇 제어 토픽을 publish하지 않는다. 기본값은 로컬 PC에서만
접속할 수 있는 `127.0.0.1`이다.

같은 신뢰 가능한 LAN의 다른 PC에서 보려면 다음처럼 명시적으로 개방한다.

```bash
ros2 launch my_robot_bringup mission_control.launch.py \
  websocket_host:=0.0.0.0 ui_host:=0.0.0.0
```

그 PC에서 `http://<로봇-PC-IP>:8080`을 연다. 현재 인증과 TLS가 없으므로
공용 인터넷에 이 포트를 노출하면 안 된다.

## WebSocket 주소 직접 지정

UI와 gateway를 별도로 실행할 때는 query parameter로 주소를 전달한다.

```text
http://localhost:8080/?ws=ws://localhost:8765/telemetry
```

UI와 gateway HTTP 서버가 서로 다른 주소라면 MJPEG 주소도 지정한다.

```text
http://localhost:8080/?ws=ws://robot:8765/telemetry&camera=http://robot:8080/camera.mjpg
```

브라우저는 ROS DDS 또는 RTAB-Map `/mapData`에 직접 접속하지 않는다.
gateway가 ROS 메시지를 아래의 작은 JSON 이벤트로 변환해야 한다.

### 전체 snapshot

```json
{
  "type": "snapshot",
  "data": {
    "pose": {"x": 1.2, "y": -0.3, "z": 0.1, "yaw": 0.7},
    "start": {"x": 0, "y": 0, "z": 0, "yaw": 0},
    "path": [{"x": 0, "y": 0, "z": 0}],
    "distanceTraveled": 1.43,
    "distanceTraveled3d": 1.51,
    "distanceSlamCorrected": 1.39,
    "distanceFromStart": 1.24,
    "mapNodes": 42,
    "battery": 87,
    "latency": 24,
    "rates": {
      "RGB": {"value": 29.8, "expected": 30, "ok": true},
      "DEPTH": {"value": 29.7, "expected": 30, "ok": true}
    }
  },
  "map": {
    "cellSize": 0.12,
    "known": [[0, 0], [1, 0], [1, 1]],
    "occupied": [[1, 1]]
  },
  "cloudPoints": []
}
```

### 증분 이벤트

```json
{"type":"pose","pose":{"x":1.3,"y":-0.2,"z":0.1,"yaw":0.75},"distanceTraveled":1.55,"distanceTraveled3d":1.61,"distanceSlamCorrected":1.49,"distanceFromStart":1.32}
{"type":"path","points":[{"x":0,"y":0,"z":0},{"x":1.3,"y":-0.2,"z":0.1}]}
{"type":"map_cells","cells":[[2,1],[2,2]],"mapNodes":43}
{"type":"rates","rates":{"RGB":{"value":29.8,"expected":30,"ok":true}}}
{"type":"event","level":"warning","message":"사람 후보 A-01 감지"}
```

실제 gateway는 다음 이벤트도 보낸다.

```json
{"type":"map","map":{"cellSize":0.12,"known":[[0,0]],"occupied":[[1,0]]},"mapNodes":42}
{"type":"cloud","points":[[1.2,-0.3,0.8],[1.3,-0.3,0.9]]}
{"type":"camera","mime":"image/jpeg","data":"<base64>","stamp":1786280000.0}
{"type":"segment_poses","poses":[{"x":0.0,"y":0.0,"z":0.1,"yaw":0.0}]}
```

- `/map`이 있으면 2D의 탐색 영역과 점유 셀을 표시한다.
- `/cloud_map`이 있으면 3D 탭에 RGB가 보존된 실제 점군을 최대 50,000점까지 표시한다.
- 3D 탭은 WebGL로 그리며 좌클릭 회전, 우클릭 이동, 휠 확대/축소와
  TOP/FRONT/SIDE/FIT 보기를 지원한다. 외부 CDN은 사용하지 않는다.
- D435 RGB는 기본 20 FPS, 640 px, JPEG 품질 70의 별도 MJPEG로 표시한다.
- `/snake/segment_poses`는 향후 3개 IMU와 관절 상태로 추정한 뱀 몸체를
  표시하기 위한 확장 입력이다. 현재 자세 추정 노드는 아직 구현하지 않았다.

주행거리는 `/visual_odom`을 2.5 cm 공간 deadband와 1.0 m/s 물리 속도
제한으로 필터링한다. UI의 `필터 주행거리`는 XY 누적값이고, `3D 주행거리`는
경사와 높이 변화를 포함하며, `SLAM 보정거리`는 `/mapPath`의 루프 폐쇄 보정
결과다. IMU 가속도를 이중 적분하지 않는다.

## 사람 말단부 YOLO Pose

사람 검출은 `/perception/person_detection` JSON만 UI로 전달한다. 공식
`yolo26n-pose.pt`는 클래스가 `person` 하나뿐이고, 앱은 17개 COCO 키포인트
중 손목과 발목이 일정 신뢰도 이상 보이는 사람만 표시한다. 자동차나 의자 등
다른 객체는 전달하지 않는다.

새 x86 PC에서 한 번만 CPU 전용 환경을 만든다. 현재 개발 PC에는 이미
`.venv-yolo`와 공식 nano Pose 모델이 준비되어 있다.

```bash
cd ~/ros2_ws
sudo apt install python3-pip python3-venv
python3 -m venv --system-site-packages .venv-yolo
.venv-yolo/bin/pip install torch torchvision \
  --index-url https://download.pytorch.org/whl/cpu
.venv-yolo/bin/pip install 'numpy<2' ultralytics
.venv-yolo/bin/python -c 'from ultralytics import YOLO; YOLO("yolo26n-pose.pt")'
```

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source .venv-yolo/bin/activate
source install/setup.bash

ros2 launch my_robot_bringup mission_control.launch.py \
  enable_person_detection:=true \
  yolo_model:=$PWD/yolo26n-pose.pt \
  yolo_device:=cpu yolo_image_size:=416 yolo_rate:=5.0
```

추론은 별도 작업 스레드에서 최신 카메라 프레임 하나만 유지하므로 처리가
느려져도 카메라 큐가 누적되지 않는다. 2회 연속 검출해야 표시하고 3회 연속
미검출해야 해제해 UI 깜빡임도 억제한다.

COCO Pose는 손목과 발목까지만 제공하고 손가락 관절은 제공하지 않는다.
손가락만 보이는 조난자를 찾으려면 손/손가락 이미지로 별도 YOLO Detect
모델을 학습해야 한다. 커스텀 모델 클래스 이름을 `hand`, `finger`, `foot`으로
만들면 같은 노드에 바로 연결할 수 있다.

```bash
ros2 launch my_robot_bringup person_pose.launch.py \
  model:=/path/to/hand_finger_best.pt device:=cpu
```

Jetson Orin Nano에서는 JetPack 호환 PyTorch를 사용하고 `device:=0`으로
실행한다. 최종 배포에서는 해당 Jetson에서 TensorRT `*.engine`을 생성하고
같은 `model` 인자에 지정한다. x86 PC에서 생성한 엔진을 Jetson으로 복사하면
안 된다.

비상정지 버튼은 시각 시제품일 뿐 실제 제어 명령을 보내지 않는다.
