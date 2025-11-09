# SmartFuelApp

# 🚗 SmartFuelApp — 스마트 주유 결제 Flutter 앱

Flutter 기반의 **스마트 주유 결제 애플리케이션**으로,  
사용자가 **카카오 로그인 → 결제 → 주유 → 완료** 과정을 한 번에 진행할 수 있는  
**Smart Fuel Robot 시스템의 프론트엔드 앱**입니다.

---

## 🌟 주요 기능

### 🔐 **카카오 로그인 연동**
- 카카오 SDK를 이용한 **OAuth2.0 로그인** 지원  
- 로그인 후 사용자 프로필(이름, 이메일, 카카오 ID) 자동 불러오기  
- 로그인 상태를 로컬 저장소(`shared_preferences`)에 유지  
- FastAPI 서버에 사용자 정보 전달 (결제 기록 연동 가능)

### 💳 **결제 기능**
- Flutter → FastAPI 서버로 결제 요청 및 승인 처리  
- 주문 정보(JSON):  
  ```json
  {
    "orderId": "TEST-001",
    "fuelType": "gasoline",
    "amount": 30
  }

카드/모바일 결제 UI 구성
결제 완료 시 로봇 제어 프로세스 자동 시작

🤖 ROS2 연동
FastAPI 서버 → ROS2 토픽 /fuel_task/start 퍼블리시
로봇 노드(MotionController, FuelTaskManager)와 실시간 통신
주유 진행 상태(/fuel_status) 수신 → Flutter UI에 표시

🧠 실시간 상태 표시
주유 단계별 상태(대기 → 진행 → 완료 → 오류) 표시
ROS2 토픽 기반 실시간 업데이트

🖥️ UI 구성
카카오 로그인 화면
유종 선택 및 결제 입력
결제 진행 애니메이션
주유 진행 및 완료 알림 화면

🧩 시스템 아키텍처
[Flutter App]
 ├─ Kakao Login
 ├─ Payment (FastAPI)
 ├─ Status Monitor
   ↓
[FastAPI Server]
   ↓
[ROS2 Nodes]
 ├─ fuel_task_manager
 ├─ motion_controller
   ↓
[Doosan E0509 + RH-P12-RN(A) Gripper + RealSense Camera]

🔗 주요 통신 프로토콜

방향	경로	설명
Flutter → Kakao	OAuth2.0 API	로그인 인증
Flutter → FastAPI	POST /start_fuel	결제 완료 후 주유 명령 송신
FastAPI → ROS2	/fuel_task/start	주유 시작 신호
ROS2 → Flutter	/fuel_status	주유 진행 상태 전달

🛠️ 개발 환경

Flutter 3.x

Dart 3.x

Kakao Flutter SDK

FastAPI 0.110+

ROS2 Humble

Ubuntu 22.04 LTS

Doosan Robotics E0509 + RH-P12-RN(A)

Intel RealSense Depth Camera