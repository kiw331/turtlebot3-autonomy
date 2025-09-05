# 🚗 TurtleAutonomy — Gazebo 자율주행 시뮬레이션 + 실환경 모바일 매니퓰레이터

>Gazebo 기반 자율주행 시뮬레이션  
차선 추종 및 모바일 매니퓰레이터 제어




## ✨ 개요

* Gazebo와 실환경 TurtleBot3를 동일 파이프라인으로 연결한 **차선 추종 자율주행**
* **ArUco 마커**로 목표 위치 추정, **MoveIt**으로 매니퓰레이터 **Pick-and-Place**
* CLAHE·BEV·PD 제어 등 영상/제어 모듈을 **재사용 가능한 노드**로 구성
* GUI와 파라미터 통합으로 **시뮬–실환경 전환의 재현성** 확보

---

## 📦 구성 개요

* **주요 기능**

  * 차선 인식 기반 자율주행 (CLAHE → Histogram Stretch → HSV 마스크 → **BEV** → 중앙선 추출 → **PD 제어**)
  * ArUco 마커 인식(rvec/tvec)과 자세 추정, 매니퓰레이터 **Pick-and-Place**
  * 시뮬레이션(Gazebo) ↔ 실환경(TurtleBot3 + Manipulator) **동일 파이프라인 운용**
  * (선택) GUI로 모드 설정(Eco/Comfort/Sport) 및 로그 뷰

* **기술 스택**

  * ROS 2 Humble, Python 3.10, Ubuntu 22.04
  * OpenCV(+ArUco), RViz, Gazebo, MoveIt
  * TurtleBot3 Waffle, OpenMANIPULATOR-X, Logitech C270
  * (실험) message\_filters로 카메라 프레임 **ApproximateTimeSynchronizer**




## ▶️ 실행 방법

### 0) 의존성 설치

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-msgs ros-humble-turtlebot3-simulations
sudo apt install ros-humble-moveit ros-humble-joint-state-publisher-gui
pip install opencv-contrib-python numpy
```

### 1) 워크스페이스 빌드

```bash
mkdir -p ~/rokey_project/Rokey/Lane_tracking_auto_driving/src
cd ~/rokey_project/Rokey/Lane_tracking_auto_driving
colcon build
source install/setup.bash
```

### 2) 시뮬레이션(Gazebo)

```bash
ros2 launch launch/sim.launch.py          # Gazebo 맵+로봇+노드 구동
# 또는 단계 실행
ros2 run turtlebot3_autorace_detect detect_lane      # 라인 인식
ros2 run turtlebot3_autorace_driving control_lane    # 자율주행 제어
```

### 3) 실환경(TurtleBot3 + Manipulator)

```bash
# 카메라 파이프라인
ros2 run turtlebot3_autorace_camera img_publish
ros2 run turtlebot3_autorace_camera image_compensation

# 라인 트래킹
ros2 run turtlebot3_autorace_detect detect_lane
ros2 run turtlebot3_autorace_detect detect_stop_line
ros2 run turtlebot3_autorace_driving control_lane

# ArUco + Pick&Place
ros2 run aruco_yolo turtlebot_aruco
ros2 run aruco_yolo pick_and_place
```

<!--

> (Gazebo 실행 스크린샷)
> (실환경 주행/조작 사진 또는 GIF)
> (ArUco 인식 예시 이미지: 마커, 좌표축, rvec/tvec)
-->

---

## 🔁 노드 & 토픽

* **lane\_detector.py**: `/camera/image_raw` → `/lane_center`
* **lane\_controller.py**: `/lane_center` → `/cmd_vel`
* **turtlebot\_aruco.py**: `/camera/image_raw` → `/detected_markers`
* **pick\_and\_place.py**: `/detected_markers` → MoveIt 액션/서비스 호출

**주요 토픽**

* `/image_compensated/compressed` — 보정된 카메라 이미지
* `/lane_center` — 라인 중앙 좌표
* `/control/cmd_vel` — 주행 제어 명령
* `/detected_markers` — ArUco ID/포즈

---

## 🧪 핵심 알고리즘/세부기술

* **차선 인식**: CLAHE(지역 대비 향상) → Histogram Stretch(전체 보정) → HSV 마스크 → **BEV** → 폴리피팅 중심선
* **주행 제어**: 중앙선–프레임 중심 오차 기반 **PD 제어**(직진/회전 가중치 분리)
* **마커 포즈 추정**: 카메라 내·외부 파라미터 기반 `rvec/tvec` → Rodrigues → 오일러 각
* **통합 안정화**: `message_filters.ApproximateTimeSynchronizer`로 프레임 정합

---

## 📈 결과

* Gazebo에서 **차선·표지판·차단바** 시나리오 통합 검증
* 실환경에서 **차선 추종 + ArUco 기반 Pick-and-Place** 시연 성공
* 다양한 조도에서 차선 인식률 향상, 프레임 동기화로 처리 안정성 확보


<!--
> (결과 비교 이미지: 시뮬레이션 검출 결과 vs 실환경 검출 결과)
> (Pick\&Place 시퀀스 다이어그램/타임라인)
-->

## 🎬 데모
<!--
* Gazebo 시뮬레이션: [https://github.com/user-attachments/assets/6378f33f-8f00-4a66-a2dc-0da84ff88955](https://github.com/user-attachments/assets/6378f33f-8f00-4a66-a2dc-0da84ff88955)
* 실환경 주행(매니퓰레이터 포함): [https://github.com/user-attachments/assets/4f77d3ca-bcff-42cb-8871-0b32b93b8788](https://github.com/user-attachments/assets/4f77d3ca-bcff-42cb-8871-0b32b93b8788)
-->


<!--

## 📚 문서

* [`docs/프로젝트3(가제보).pdf`](docs/프로젝트3%28가제보%29.pdf)
* [`docs/프로젝트3(실환경).pdf`](docs/프로젝트3%28실환경%29.pdf)


-->



## 발표자료


## Reference
- https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

