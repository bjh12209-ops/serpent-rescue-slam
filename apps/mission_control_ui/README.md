# Mission Control UI prototype

재난구조 뱀 로봇의 PC 브라우저 관제 화면 시제품이다. 별도 패키지 설치 없이
mock telemetry로 동작하며, ROS 2에 직접 접속하지 않는다.

## 실행

```bash
cd ~/ros2_ws
python3 -m http.server 8080 --directory apps/mission_control_ui
```

브라우저에서 `http://localhost:8080`을 연다. 지도 드래그, 휠 확대/축소,
로봇 추적, 2D·3D 동시 `OVERVIEW`와 3-IMU `SNAKE POSE` 화면을 사용할 수 있다.

## 실제 ROS 2 연결

먼저 한 터미널에서 SLAM을 실행한 뒤, 다른 터미널에서 gateway를 실행한다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup mission_control.launch.py
```

기본 PC 프로필은 2D 셀 5 cm, 유효 3D 점 최대 60,000개,
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
    },
    "imus": {
      "50": {"online": true, "role": "HEAD", "quaternion": {"x": 0, "y": 0, "z": 0, "w": 1}}
    },
    "snakeModel": {"moduleCount": 7, "activeModules": []}
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

실제 gateway는 다음 이벤트도 보낸다. 3D 점군은 큰 JSON 배열 대신
`SPC1 + uint32 count + float32 XYZ/RGB` 이진 프레임으로 보낸다.

```json
{"type":"map","map":{"cellSize":0.12,"known":[[0,0]],"occupied":[[1,0]]},"mapNodes":42}
{"type":"camera","mime":"image/jpeg","data":"<base64>","stamp":1786280000.0}
{"type":"segment_poses","poses":[{"x":0.0,"y":0.0,"z":0.1,"yaw":0.0}]}
{"type":"imu","imu":{"sensorId":"50","displayId":"0x50","role":"HEAD","online":true,"quaternion":{"x":0,"y":0,"z":0,"w":1},"eulerDeg":{"roll":0,"pitch":0,"yaw":0}}}
```

- `/map`이 있으면 2D의 탐색 영역과 점유 셀을 표시한다.
- `/cloud_map`이 있으면 RGB가 보존된 실제 점군을 최대 60,000점까지 표시한다.
- gateway는 점군을 기본 3 cm voxel 캐시에 병합한다. 비거나 부분적인 RTAB-Map
  출력은 기존 캐시를 지울 수 없고 브라우저 GPU 버퍼도 빈 값으로 교체하지 않는다.
- `/mapGraph`는 노드 수 표시에만 사용한다. gateway가 실제 재시작할 때 생성되는
  `sessionId`만 새 점군 세션을 구분하므로 loop closure를 초기화로 오판하지 않는다.
- 기본 `OVERVIEW`에는 2D 지도와 3D 점군이 함께 나오며 오른쪽 전방 카메라와
  사람 전용 탐지 결과도 계속 표시된다.
- 3D 점군은 WebGL 대신 CPU depth-buffer로 완성 프레임을 만든 뒤 교체한다.
  계산 중에는 직전 프레임을 유지하며 좌클릭 orbit, 우클릭/Shift pan, 휠 zoom과
  TOP/FRONT/SIDE/FIT/RESET 보기를 지원한다. `SNAKE POSE` WebGL은 해당 탭을
  선택할 때만 생성한다.
- D435 RGB는 최대 20 FPS, 640 px, JPEG 품질 65의 별도 MJPEG로 표시한다.
- `/imu_50/data`, `/imu_51/data`, `/imu_52/data`의 쿼터니언은 각각 HEAD,
  MIDDLE, TAIL 모듈을 구동한다. `SNAKE POSE`에서 모듈 박스, 연결선과 RGB
  local axis를 표시한다. 0x51/0x52 모듈 간격은 측정 전까지 UI 전용이다.
- `/snake/segment_poses`는 향후 관절 구조까지 반영한 estimator를 위한 기존
  확장 입력으로 유지한다.

주행거리는 `/visual_odom`을 2.5 cm 공간 deadband와 2.5 m/s 물리 속도
제한으로 필터링한다. UI의 `필터 주행거리`는 XY 누적값이고, `3D 주행거리`는
경사와 높이 변화를 포함하며, `SLAM 보정거리`는 `/mapPath`의 루프 폐쇄 보정
결과다. IMU 가속도를 이중 적분하지 않는다.

## 사람 말단부 YOLO Pose

사람 검출은 `/perception/person_detection` JSON만 UI로 전달한다. 공식
`yolo26n-pose.pt`는 클래스가 `person` 하나뿐이다. 기본 모드는 사람 몸체가
보이면 표시하고, 보이는 손목·발목 키포인트도 함께 전달한다. 자동차나 의자 등
다른 객체는 전달하지 않는다. `yolo_require_extremity:=true`를 주면 손목 또는
발목이 보이는 후보만 통과시킬 수 있다.

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
source install/setup.bash

ros2 launch my_robot_bringup mission_control.launch.py \
  yolo_model:=$PWD/yolo26n-pose.pt \
  yolo_device:=cpu yolo_image_size:=320 yolo_rate:=2.0
```

gateway가 원본을 320 px, 2 Hz `/mission_control/yolo_input`으로 축소한다.
추론은 별도 작업 스레드에서 최신 프레임 하나만 유지하므로 처리가
느려져도 카메라 큐가 누적되지 않는다. YOLO 가상환경과 모델은 알려진 작업공간
경로에서 자동 탐색한다. 2회 연속 검출해야 표시하고 3회 연속
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
