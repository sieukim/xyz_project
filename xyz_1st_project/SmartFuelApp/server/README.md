# SmartFuel — ROS2 HTTP bridge (FastAPI)

이 디렉토리는 Flutter 앱에서 `POST /start_fuel` 를 보내면 ROS2로 주유 시작 명령을 전달하고,
`GET /status/{orderId}` 로 진행 상태를 조회할 수 있는 간단한 HTTP 서버(프로토타입)를 포함합니다.

주의: ROS2(rclpy)는 일반적으로 시스템 패키지로 설치되며 pip로 설치하지 않습니다. 서버를 ROS2 머신에서 실행하려면
ROS2 환경을 활성화한 상태에서 실행하세요.

설치 (Windows PowerShell 예시)

```powershell
# 가상환경 생성
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# FastAPI/uvicorn 설치
pip install -r requirements.txt

# (옵션) ROS2 환경을 활성화: ROS2 설치 경로에 따라 다름
# 예: call C:\dev\ros2\local_setup.bat

# 서버 실행
uvicorn server:app --host 0.0.0.0 --port 5000
```

테스트

```powershell
# 결제(앱에서 보내는 JSON 예시)
curl -X POST http://localhost:5000/start_fuel -H "Content-Type: application/json" -d '{"orderId":"order-123","fuelType":"휘발유","amount":50000}'

# 상태 조회
curl http://localhost:5000/status/order-123
```

통합 가이드
- Flutter 쪽 `rosServerUrl` 를 `http://<ROS2_PC_IP>:5000/start_fuel` 로 설정하세요.
- 실제 ROS 노드가 상태를 publish한다면 서버의 `_publish_to_ros2` 또는 `background_start` 를 수정해
  그 토픽을 구독하여 `orders[...]` 상태를 업데이트하도록 구현하면 됩니다.
