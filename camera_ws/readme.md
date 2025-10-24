publisher터미널에서 카메라 키는 명령어
$ ros2 run camera_state_pubsub camera_state_publisher --ros-args -p device_index:=0
# 카메라 번호에 맞게 설정

Subscriber터미널에서 카메라 신호 받는 명령어
$ ros2 run camera_state_pubsub camera_state_subscriber

yolo모델 yolo_viewer받아오는 명령어 (위랑 same)
$ ros2 run camera_state_pubsub yolo_viewer --ros-args -p window:=false



