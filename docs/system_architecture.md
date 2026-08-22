# D435·IMU SLAM 및 웹 관제 구조

## 범위

이 저장소는 다음 데이터 흐름까지만 담당한다.

```text
D435 RGB + aligned depth -> RGB-D visual odometry --┐
WT901C485 0x50 head -> orientation/angular velocity --┤
WT901C485 0x51/0x52 -> module pose visualization      │
                                                     v
                                         robot_localization EKF
                                                     |
                                          odom -> base_link
                                                     |
                          RGB-D + odometry -> RTAB-Map
                                                     |
                   map / cloud / pose / path / distance / diagnostics
                                                     |
                                  read-only ROS 2 gateway
                                                     |
                                    browser Mission Control UI
```

모터, 조이스틱, 주행 정책과 액추에이터 명령은 이 저장소의 책임이 아니다.

## TF 소유권

```text
map --RTAB-Map--> odom --EKF--> base_link
                                  └── camera_link
                                        └── imu_50_link (-0.05, 0, +0.02 m)
```

- RTAB-Map만 `map -> odom`을 발행한다.
- 융합 모드에서는 EKF만 `odom -> base_link`를 발행한다.
- 카메라 전용 모드에서는 RGB-D odometry가 해당 TF를 발행한다.
- 최종 장착 후 실제 센서 위치와 회전으로 고정 TF를 다시 측정한다.

## 주요 ROS 인터페이스

| 데이터 | ROS 인터페이스 | 소비자 |
|---|---|---|
| D435 RGB | `/camera/camera/color/image_raw` | VO, RTAB-Map, gateway 웹 카메라 |
| YOLO 입력 | `/mission_control/yolo_input` | 320 px/2 Hz person detector |
| aligned depth | `/camera/camera/aligned_depth_to_color/image_raw` | VO, RTAB-Map |
| head WT901 IMU | `/imu_50/data` | EKF, gateway |
| middle/tail WT901 IMU | `/imu_51/data`, `/imu_52/data` | gateway body-pose view |
| visual odometry | `/visual_odom` | EKF, 텔레메트리 |
| filtered odometry | `/odometry/filtered` | RTAB-Map, 텔레메트리 |
| 2D map | `/map` | RViz, gateway |
| 3D map | `/cloud_map`, `/mapData` | RViz, gateway |
| current pose | `/slam/current_pose` | RViz, gateway |
| path | `/slam/path` | RViz, gateway |
| distance/rates | `/slam/*` | gateway |
| person event | `/perception/person_detection` | gateway/UI |

## 웹 전송 경계

브라우저는 ROS DDS에 직접 연결하지 않는다. gateway가 ROS 메시지를
WebSocket JSON과 MJPEG로 변환한다.

- 위치·방향·거리·Hz: 작은 JSON 이벤트
- 경로: 간소화한 점 목록
- 2D 지도: 낮은 빈도의 점유 셀
- 3D 지도: gateway 세션 동안 유지되는 voxel/downsample 점군 캐시
- RGB: 오래된 프레임을 쌓지 않는 latest-frame-only MJPEG
- 사람 감지: person-only 이벤트와 신뢰도

gateway는 읽기 전용이며 로봇 제어 토픽을 발행하지 않는다. 인증과 TLS가
없으므로 기본값 `127.0.0.1`을 유지하고, 신뢰 가능한 LAN에서만
`0.0.0.0`으로 개방한다.

## 품질 기준

- D435 RGB/depth와 VO는 약 30 Hz를 목표로 한다.
- WT901과 EKF는 실제 RS485 여유를 고려해 약 45 Hz를 목표로 한다.
- RTAB-Map 지도 갱신은 CPU 부하를 고려해 2 Hz로 운용한다.
- 시작점 재방문 시 loop closure가 수락돼야 한다.
- 지도, 현재 pose와 실제 경로가 같은 `map` 좌표에서 일치해야 한다.
- 카메라나 점군 지연이 WebSocket 큐에 누적되지 않아야 한다.
- RTAB-Map의 부분 출력과 graph 최적화는 이미 표시 중인 3D GPU 버퍼를 지우지
  않는다. 새 gateway `sessionId`만 새로운 점군 세션을 시작한다.

세부 실행 및 진단 순서는 [지도 품질 가이드](mapping_quality.md)를 따른다.
