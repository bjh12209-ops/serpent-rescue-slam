# 재난구조 로봇 앱 UI 구상

## 핵심 경험

화면은 어두운 미지의 지도에서 시작한다. 로봇이 관측한 영역만 실시간으로
밝아지며, 사용자는 게임의 월드맵처럼 출발점, 현재 위치·방향, 실제 동선과
발견한 사람을 한눈에 본다.

RViz 화면을 영상으로 캡처해 전송하지 않는다. SLAM 결과를 경량화해 앱이
직접 렌더링해야 낮은 대역폭에서도 부드럽고 터치 조작이 가능하다.

## 화면 배치

```text
┌────────────────────────────────────────────────────────────┐
│ MISSION 01  │ SLAM 정상 │ 배터리 │ 연결 지연 │ 탐색 08:42 │
├───────────────────────────────────────┬────────────────────┤
│                                       │ RGB/탐지 카메라     │
│       어두운 미지 영역                ├────────────────────┤
│          ╭─ 탐색된 3D/2D 지도         │ 현재 위치 x/y/yaw  │
│   START ●┼──── 실제 동선 ────▶ ROBOT  │ 출발점 거리        │
│          ╰─ 관측하며 밝아지는 영역     │ 누적 주행거리       │
│                                       │ RGB 30 / Depth 30  │
│                                       │ VO 30 / SLAM 3     │
│                                       │ IMU 45 / EKF 45   │
├───────────────────────────────────────┴────────────────────┤
│ 2D MAP │ 3D MAP │ CAMERA │ TARGETS      [정지] [복귀]     │
└────────────────────────────────────────────────────────────┘
```

모바일에서는 지도 전체 화면 위에 카메라와 상태 카드를 접을 수 있는 overlay로
배치한다. 기본 화면은 위에서 본 2D 지도와 영구 캐시된 3D 점군을 좌우로 동시에
표시하며, 오른쪽 열의 전방 카메라와 사람 탐지 결과도 항상 유지한다. 각 지도는
단독 확대 화면으로 전환할 수 있다.

## 지도 표현

- 미탐색 영역: 검정 또는 짙은 남색
- 탐색된 빈 공간: 어두운 회색
- 벽·잔해: RGB-D 색상 또는 높이에 따른 색상
- 출발점: 녹색 원과 `START`
- 현재 로봇: 진행 방향이 보이는 주황색 화살표
- 실제 동선: 밝은 녹색 polyline
- 계획 경로: 점선 파란색 polyline
- 사람 후보: 노란색, 확인된 사람: 빨간색 marker
- 오래된 map tile과 새 map tile 사이에는 짧은 fade를 적용

## ROS 입력과 앱 데이터

| ROS 입력 | gateway 출력 | 권장 화면 갱신 |
|---|---|---:|
| `/slam/current_pose` | 현재 `x,y,z,quaternion` | 10~20 Hz |
| `/slam/start_pose` | 출발점 | 최초/변경 시 |
| `/slam/path` | 간소화된 실제 동선 증분 | 2~5 Hz |
| `/slam/distance_traveled` | 누적 주행거리 m | 2~5 Hz |
| `/slam/distance_from_start` | 출발점 직선거리 m | 2~5 Hz |
| `/slam/diagnostics` | 각 스트림 Hz/age/상태 | 1 Hz |
| `/grid_map` | 12 cm로 축약한 점유 셀 | 1 Hz |
| `/cloud_map` | 4 cm voxel 캐시에 병합한 최대 30,000점 이진 3D 점군 | 1 Hz |
| D435 RGB | 640 px latest-only MJPEG | 최대 20 Hz |

`/mapData` 전체를 브라우저로 반복 전송하지 않는다. 현재 gateway는
`/map`을 5 cm 셀로 합치고 `/cloud_map`을 최대 30,000점으로 제한해
WebSocket으로 전달한다. 지도가 커져 전체 셀 전송이 부담되는 시점에는 변경
tile만 보내는 계약으로 확장한다.

## 구현 단계

1. 현재 RViz 탑다운 화면으로 UI 의미와 색상 검증
2. PC 브라우저에서 가짜 telemetry로 고정 UI 제작 — 현재 시제품 구현됨
3. 실제 pose/path/diagnostics WebSocket 연결 — 구현됨
4. 2D 점유격자와 다운샘플링 전송 연결 — 구현됨
5. 3D cloud와 카메라 overlay 연결 — 구현됨
6. 휴대폰 브라우저 성능·재접속·저대역폭 검증
7. 마지막에 PWA 또는 네이티브 shell로 패키징

현재 시제품은 `apps/mission_control_ui`에 있으며 아래 명령으로 실행한다.

```bash
python3 -m http.server 8080 --directory apps/mission_control_ui
```

실제 ROS 2 연결은 `my_robot_bringup/mission_control_gateway.py`가 담당한다.
`ros2 launch my_robot_bringup mission_control.launch.py`를 실행하면 UI 서버와
읽기 전용 WebSocket gateway가 함께 시작된다.

향후 3개 IMU 기반 몸체 자세 추정기는 `/snake/segment_poses`
(`geometry_msgs/PoseArray`)를 publish한다. gateway와 UI는 이미 이 배열을
연결된 몸체 선분으로 표시할 수 있다. IMU 방향만 적분해 위치를 만들지 않고
관절 엔코더와 뱀 로봇 기구학 제약을 함께 사용해야 한다.

최종 앱 전에 연결 끊김, 지도 갱신 지연, SLAM lost와 안전정지가 명확히
구분되어 보여야 한다. 앱의 지도 렌더링 실패가 로봇 제어를 멈추거나 반대로
로봇 안전정지를 숨기지 않도록 제어와 표시 경로를 분리한다.
