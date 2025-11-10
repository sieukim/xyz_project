# xyz_1st_project

## 프로젝트 주제: 자동 주유 로봇

팀원: 심승환, 김민서, 안동환</br>
기간: 2025.10.20 ~ 2025.10.31</br>
시나리오</br>
- 차량이 들어오면 객체인식(yolov8)을 통해 차량의 종류를 인식한다.</br>
- 그 후 앱을 통해 원하는 금액을 결제
- 결제가 완료되면 로봇암이 움직여 주유를 시작
- ros 명령어로 로봇암 제어
    -> 이때 movej, movel, 강성제어를 통해 로봇암 제어
- isaac sim을 통해 환경구축


publisher터미널에서 카메라 키는 명령어
$ ros2 run camera_state_pubsub camera_state_publisher --ros-args -p device_index:=0
# 카메라 번호에 맞게 설정

Subscriber터미널에서 카메라 신호 받는 명령어
$ ros2 run camera_state_pubsub camera_state_subscriber

yolo모델 yolo_viewer받아오는 명령어 (위랑 same)
$ ros2 run camera_state_pubsub yolo_viewer --ros-args -p window:=true


df_doosan =>  세미 프로젝트 하드코딩
df_doosan2 => 1차 프로젝트 코드
