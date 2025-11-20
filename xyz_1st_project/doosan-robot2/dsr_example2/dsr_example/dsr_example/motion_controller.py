#!/usr/bin/env python3
"""
MotionController (combined)
- FuelTaskManager 기능(결제 신호/FSM/차량 감지 연동)과 MotionController(로봇/그리퍼/안전이동)를 하나의 노드로 통합
- 외부(Flutter→FastAPI)에서는 기존처럼 /fuel_task/start 토픽으로 JSON을 퍼블리시하면 됩니다.

주요 토픽/서비스
- sub  : /fuel_task/start (String JSON: {orderId, fuelType, amount})
- sub  : /car_detected (String "detected")
- sub  : /fuel/yolo_detections (String JSON array)  # YOLO 결과(웹캠/리얼센스) 통합 입력
- sub  : /fuel/object_3d (PointStamped, camera frame)  # 리얼센스 기반 3D 타깃 포인트
- sub  : /stop_motion (Bool)
- pub  : /fuel_status (String: idle/progress/done/error)
- pub  : /target_direction (Float32)  # (선택) 노즐↔주유구 방향 보조
- srv  : /motion_controller/orient_negative_y (Trigger)

설정 포인트
- CAMERA_OFFSET_TCP_Z_M = +0.05  # 카메라가 TCP보다 5 cm 위
- ORIENT_PRESET_POSJ : 툴을 -Y(바닥 방향)으로 보는 자세 프리셋

주의
- 실제 환경에 맞게 워크스페이스/최소Z/속도·가속도 상한 등을 조정하세요.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger

import numpy as np
import json
import time
# import threading
from collections import deque

import DR_init
from dsr_example.gripper_drl_controller import GripperController
from dsr_example.llm_narrator_client import AINarrator

# ─────────────────────────────────────────────────────────────
# Doosan 기본 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ─────────────────────────────────────────────────────────────
# 안전/동작 파라미터
TARGET_LABEL = "green_car"      # YOLO 허용 라벨(예: 자동차)
CAP_LABEL = "white_cap"          # YOLO 허용 라벨(예: 자동차)
NOZZLE_LABEL = "nozzels"         # YOLO 허용 라벨(예: 자동차)
LABEL_TIMEOUT_SEC = 3.0          # 허용 라벨 감지 유지 시간
V_MAX = 60                       # 이동 속도 상한 (Doosan 단위)
A_MAX = 60                       # 가속도 상한
PRE_UP_MM = 120.0                # 접근 전 위로 확보할 높이
STANDOFF_MM = 120.0              # 목표 지점 위에서 멈출 여유
MIN_Z_MM = 400.0                 # 절대 최소 Z (충돌 방지)
WS_XY_MM = 800.0                 # XY 워크스페이스 절대 한계(±)

CAMERA_OFFSET_TCP_Z_M = 0.05     # 카메라가 TCP보다 +5 cm (위)
ORIENT_PRESET_POSJ = (20, 35, 105, 105, -90, 50)  # 바닥(-Y) 방향 프리셋

ORIENT_POSJ_POS_XL = (10, 40, 80, 20, -30, 70) # 바닥(+X) 방향 프리셋 : 휘발유
ORIENT_POSJ_POS_XR = (-15, 40, 80, 150, 30, -60) # 바닥(+X) 방향 프리셋 : 경유

# ─────────────────────────────────────────────────────────────
class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        self.get_logger().info("🤖 MotionController (combined) starting...")
        self.coord_buffer = deque(maxlen=10)  # 최근 10개 좌표 유지
        self.last_valid_coord = None           # 최근 안정 좌표
        
        try:
            self.gripper = GripperController(node=self, namespace=ROBOT_ID)
            if not self.gripper.initialize():
                raise RuntimeError("Gripper initialization failed")

        except Exception as e:
            self.get_logger().error(f"❌ Gripper/Init error: {e}")
            raise

        # 구독/퍼블리셔
        self.sub_start = self.create_subscription(String, '/fuel_task/start', self.on_task_start, 10)
        self.sub_car_detected = self.create_subscription(String, '/car_detected', self.on_car_detected, 10)
        self.sub_yolo = self.create_subscription(String, '/fuel/yolo_detections', self.on_detections, 10)
        self.sub_obj3d = self.create_subscription(PointStamped, '/fuel/object_3d', self.on_point, 10)
        # self.sub_stop = self.create_subscription(Bool, '/stop_motion', self.on_stop_signal, 10)
        self.sub_webcam = self.create_subscription(String, '/fuel/webcam_detections', self.on_webcam_detections, 10)
        self.sub_realsense = self.create_subscription(String, '/fuel/realsense_detections', self.on_realsense_detections, 10)
        
        self.pub_status = self.create_publisher(String, '/fuel_status', 10)
        self.pub_gripper = self.create_publisher(Float32, '/fuel/gripper_move', 10)

        # 초기화
        self._init_gripper_and_home()

        self.narrator = AINarrator(self, use_llm=True, llm_url="http://localhost:8001/narrate")

        # 기본 Hand–Eye 행렬 설정 (fuel_cap 모드 기본)
        self.mode = "fuel_cap"

        self.timer = self.create_timer(0.5, self.control_loop)

    # ─────────────────────────────────────────────────────────
    # 초기화 및 유틸
    def _init_gripper_and_home(self):
        from DSR_ROBOT2 import wait, movej
        self.get_logger().info("홈 자세 이동")
        movej([0, 0, 90, 0, 90, 0], 60, 60)
        wait(2.0)

        # self.gripper_move(0)
        # self.gripper_move(700)
        # self.gripper_move(500)
        # self.gripper_move(200)
        # self.gripper_move(0)

        # === 추가되는 초기화 변수들 ===
        self.ORIENT_FUEL_POS = None
        self.ORIENT_GUN_POS = None
        self.ORIENT_CAP_POS = None
        
        # FSM/주문 상태
        self.current_state = "IDLE"  # IDLE → PROGRESS → DONE
        self.order_id = None
        self.fuel_type = None
        self.amount = 0.0

        # 감지 상태
        self.last_label_ts = 0.0
        self.allowed_label = CAP_LABEL
        self.last_car_detected_event = False
        self.start_fuel = False
        self.is_moving = False

        # -Y축에서 플래그
        self.object_task_done = False
        self.xy_centered_once = False
        self.arrived_Y = False

        # 이동 상태
        self.is_busy = False
        self.force_triggered = False
        self.reached_target_once = False

        # Narrator용 1회성 플래그
        self._narrated_xy_align = False
        self._narrated_xy_done = False
        self._narrated_depth_done = False

    # ─────────────────────────────────────────────────────────
    # 각 카메라별 감지
    def on_webcam_detections(self, msg: String):
        """웹캠 YOLO 결과 — 탐색 중단 없이 로그만 출력"""
        try:
            dets = json.loads(msg.data)
            labels = [d.get('cls') for d in dets if 'cls' in d]
            if TARGET_LABEL in labels:
                self.get_logger().info(f"👁️ [Webcam YOLO] {TARGET_LABEL} 감지됨 — 탐색 유지")
        except Exception as e:
            self.get_logger().warn(f"웹캠 YOLO 파싱 오류: {e}")

    def on_realsense_detections(self, msg: String):
        """리얼센스 YOLO 결과 — 탐색 중단 트리거 (이동은 control_loop에서 수행)"""
        try:
            dets = json.loads(msg.data)
            labels = [d.get('cls') for d in dets if 'cls' in d]

            if CAP_LABEL in labels:
                # 1️⃣ 탐색 중이면 멈춤
                if getattr(self, "searching", False):
                    self.get_logger().info(f"✅ [Realsense YOLO] {CAP_LABEL} 감지됨 — 탐색 중단")
                    self.detected_cap_once = True
                    self.last_label_ts = time.time()
    
                # 2️⃣ 좌표 갱신은 object_callback에서 처리하므로 여기서는 안 건드림
                else:
                    if getattr(self, "detected_cap_once", False):
                        return

        except Exception as e:
            self.get_logger().warn(f"리얼센스 YOLO 파싱 오류: {e}")

    # 결제/시작 신호 & 차량 감지 FSM
    def on_task_start(self, msg: String):
        """Flutter/서버에서 결제 완료 후 주유 시작 신호(JSON)를 받는다."""
        try:
            payload = json.loads(msg.data)
            self.order_id = payload.get("order_id", "UNKNOWN")
            self.payment_confirmed = True
            self.get_logger().info(f"💳 결제 완료 수신 (order_id={self.order_id})")
        except Exception as e:
            self.get_logger().error(f"❌ 결제 메시지 파싱 실패: {e}")
            return

    def on_car_detected(self, msg: String):
        if not msg.data or self.current_state == "IN_PROGRESS":
            return  # 차량이 사라졌으면 무시

        # 상태 저장
        self.detected_car = True
        self.get_logger().info("🚗 차량 감지됨")

        # 조건 확인
        if getattr(self, "payment_confirmed", False) and self.current_state != "IN_PROGRESS":
            self.get_logger().info("✅ 차량 감지 + 결제 완료 → 주유 시퀀스 시작")
            self.start_fueling_sequence()
        else:
            if not getattr(self, "payment_confirmed", False):
                self.get_logger().info("💤 결제 대기 중 (아직 결제 완료 신호 없음)")
            elif self.current_state == "IN_PROGRESS":
                self.get_logger().info("⚙️ 이미 주유 시퀀스 진행 중")

    def on_detections(self, msg: String):
        """YOLO 결과 JSON에서 허용 라벨 감지 시 타임스탬프 갱신"""
        try:
            dets = json.loads(msg.data)
            labels = [d.get('cls') for d in dets if 'cls' in d]
            if self.allowed_label in labels:
                self.last_label_ts = time.time()
        except Exception as e:
            self.get_logger().warn(f"parse det error: {e}")

    def start_fueling_sequence(self):
        if self.current_state == "IN_PROGRESS":
            return
        
        self.fuel_type = "경유" # 음성인식으로 경유 휘발유 결정

        self.get_logger().info("🚀 주유 시퀀스 시작: orient_negative_y() → 탐색 시작")

        # 🔥 Narrator 호출
        self.narrator.narrate("start_sequence", fuel_type=self.fuel_type)

        # 1️⃣ 툴 -Y(바닥) 방향 회전
        self.orient_negative_y()

    def search_for_object(self):
        """객체가 인식될 때까지 상하좌우로 10cm씩 탐색 이동하는 함수"""
        from DSR_ROBOT2 import movel, wait, DR_MV_MOD_REL
        from DR_common2 import posx as dr_posx
        import time

        self.searching = True
        self.object_task_done = False  # on_point에서 True로 바뀜
        LABEL_TIMEOUT_SEC = 1.0        # YOLO 감지 유효기간

        while rclpy.ok():
            # 1) 객체 감지 신호 유지 확인
            age = time.time() - self.last_label_ts
            if age > LABEL_TIMEOUT_SEC:
                # YOLO가 잠깐 못볼 때를 대비해 on_point() 실행은 유지
                self.get_logger().warn(
                    f"⚠️ YOLO 감지 끊김(age={age:.2f}) — 유지 중..."
                )

            # 2) 작업 완료되면 종료
            if self.object_task_done:
                self.get_logger().info("🏁 객체 작업 완료 → 탐색/감시 종료")
                self.searching = False
                return

            # spin 한번씩 돌려서 on_point() 계속 작동하도록 유지
            rclpy.spin_once(self, timeout_sec=0.1)

        self.searching = False
        self.get_logger().info("🔁 탐색/감시 루프 종료 (노드 종료)")

    # ──────────────────── 좌표 noise 제거용 ─────────────────────────
    def gripper_move(self, stroke: int, wait_sec: float = 2.0):
        """
        통합 그리퍼 제어 함수
        - GripperController 가 있으면 사용
        - 아니면 /fuel/fuel_grippermove (Float32) 토픽 사용
        """
        from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        try:
            self.get_logger().info(f"[Gripper] move({stroke}) using GripperController")
            self.gripper.move(stroke)
            from DSR_ROBOT2 import wait
            wait(wait_sec)

            # 2) fallback: Float32 퍼블리셔로 보내기
            self.get_logger().info(f"[Gripper] move({stroke}) via pub_gripper")
            msg = Float32()
            msg.data = float(stroke)
            self.pub_gripper.publish(msg)

        except Exception as e:
            self.get_logger().error(f"❌ Gripper move error: {e}")

    def smooth_coordinates(self, Xb, Yb, Zb):
        """최근 좌표 평균을 통한 이동평균 필터"""
        self.coord_buffer.append((Xb, Yb, Zb))
        if len(self.coord_buffer) < 3:
            return Xb, Yb, Zb  # 초기엔 필터 적용 X
        avg = np.mean(self.coord_buffer, axis=0)
        return avg[0], avg[1], avg[2]
    
    def filter_jump(self, Xb, Yb, Zb, threshold=0.15):
        """좌표 점프 방지: 이전 좌표 대비 급격한 변화 제거"""
        if self.last_valid_coord is None:
            self.last_valid_coord = (Xb, Yb, Zb)
            return Xb, Yb, Zb

        Xp, Yp, Zp = self.last_valid_coord
        if (abs(Xb - Xp) > threshold or
            abs(Yb - Yp) > threshold or
            abs(Zb - Zp) > threshold):
            self.get_logger().warn("⚠️ 좌표 점프 감지 → 이전 좌표 유지")
            return Xp, Yp, Zp

        self.last_valid_coord = (Xb, Yb, Zb)
        return Xb, Yb, Zb
    
    # ────────────────────────────────────────────────────────────────
    def on_point(self, msg: PointStamped):
        from DSR_ROBOT2 import movel, wait, get_current_posj, get_current_posx, DR_MV_MOD_ABS, DR_MV_MOD_REL
        from DR_common2 import posx
        try:
            age = time.time() - self.last_label_ts

            if age > 1.2 and not getattr(self, "warned_cap_missing", False):
                self.warned_cap_missing = True
                self.say("주유구 인식 불가합니다. 주유구를 열어주세요")
                # 🔥 Narrator 버전 (템플릿 기반)
                # self.narrator.narrate("cap_not_detected")
                self.get_logger().warn("⚠️ 주유구 인식 불가 — 음성 안내 출력됨")
                return

            # YOLO 다시 보이면 안내 플래그 해제
            if age <= 1.0:
                self.warned_cap_missing = False
                
            poses = get_current_posx()
            if not poses or len(poses[0]) < 6:
                return

            tcp_pose = poses[0][:6]
            # cur_x = tcp_pose[0] / 1000.0
            # cur_y = tcp_pose[1] / 1000.0
            # cur_z = tcp_pose[2] / 1000.0

            Xc, Yc, Zc = msg.point.x, msg.point.y, msg.point.z

            # ---- 튐 프레임 제거 ----
            if not (0.05 < Zc < 1.2):
                return

            # ---- XY error 계산 ----
            error_x = Xc
            error_y = Yc

            # ---- 중심정렬 완료 조건 ----
            xy_centered = (abs(error_x) < 0.005 and abs(error_y) < 0.01)

            if not self.xy_centered_once:   # 🔥 처음에만 XY 정렬 허용
                # ------------------------
                # STEP 1) XY 중심정렬
                # ------------------------
                if not xy_centered:
                    # 🔥 Narrator: XY 정렬 시작 (한 번만)
                    if not self._narrated_xy_align:
                        # self.narrator.narrate("xy_aligning")
                        self._narrated_xy_align = True

                    gain = 0.6
                    move_x = -error_x * gain
                    move_y = -error_y * gain

                    # limit
                    move_x = float(np.clip(move_x, -0.02, 0.02))
                    move_y = float(np.clip(move_y, -0.02, 0.02))

                    self.get_logger().info(
                        f"🎯 XY 정렬 중: eX={error_x:.4f}, eY={error_y:.4f} "
                        f"→ move=({move_x:.3f},{move_y:.3f})"
                    )

                    # ---- XY는 절대 이동! (Z 고정) ----
                    target = list(tcp_pose)
                    target[0] += move_x * 1000
                    target[2] += move_y * 1000
                    # target[2] = same Z

                    movel(target, v=80, a=80, mod=DR_MV_MOD_ABS)
                    return

                # ------------------------
                # STEP 2) Z 접근 시작
                # ------------------------
                self.get_logger().info("🟩 XY 중앙 정렬 완료 → 깊이 접근 시작")
                self.xy_centered_once = True
                return
            
            movel(posx(0, -80, 0, 0, 0, 0), v=80, a=80, mod=DR_MV_MOD_REL)
            wait(1.5)

            self.ORIENT_FUEL_POS = get_current_posj()
            self.check_crash(1)
            self.gripper_move(0)

            movel(posx(40, -40, 0, 0, 0, 0), v=80, a=80, mod=DR_MV_MOD_REL)
            wait(1.5)
            
            self.ORIENT_CAP_POS = get_current_posj()
            # target_depth = 0.10  # 원하는 거리
            # depth_error = Zc - target_depth
            # # 🔥 남은 거리 출력
            # self.get_logger().info(
            #     f"📏 깊이 접근 중: Zc={Zc:.4f}m, target={target_depth:.4f}m, "
            #     f"remaining={depth_error:.4f}m"
            # )
            
            # # 아직 멀면 Z만 접근
            # if depth_error > 0.01:
            #     move_z = -0.03   # 앞으로 1cm
            #     movel([0,move_z*1000,0,0,0,0], v=80, a=80, mod=DR_MV_MOD_REL)
            #     return

            # 도착
            self.arrived_Y = True
            self.get_logger().info("🟥 깊이 접근 완료")
            
        except Exception as e:
            self.get_logger().error(f"❌ error: {e}")

    def control_loop(self):
        # self.get_logger().info("✅ 주유구에 도착 완료!")
        try:
            from DSR_ROBOT2 import movej, movel, wait, get_current_posx, DR_MV_MOD_ABS, DR_MV_MOD_REL
            from DR_common2 import posx, posj
            pose = get_current_posx()[0][:6]
            
            # ✅ 단계별 동작 분리
            if self.mode == "fuel_cap":
                if not self.arrived_Y :                                                                                                                                                                                                                                                                                                                                                                                                                        
                    return
                
                movel(posx(0, 0, 0, 0, 45, 0), v=50, a=50, mod=DR_MV_MOD_REL)
                wait(1.5)

                # 🔥 Narrator: 캡 오픈 시작
                # self.narrator.narrate("cap_open_start")
                self.rotate_grip(2, True)

                movel(posx(0, 30, 30, 0, 0, 0), v=80, a=80, mod=DR_MV_MOD_REL)
                wait(1.0)
                
                movej([0, 0, 90, 0, 90, 0], 80, 80)
                wait(1.0)
                # self.orient_z_down()
                self.check_crash(2)
                self.start_fuel = True
                self.mode = "nozzle"

                # 🔥 다음 on_point를 위해 상태 리셋
                self.xy_centered_once = False
                self.arrived_Y = False
                self.object_task_done = False
                self.allowed_label = NOZZLE_LABEL  # YOLO 라벨 전환

                # 🔥 이제부터 on_point는 '주유건'을 향해 다시 정렬하게 됨
                self.get_logger().info("🟦 주유건 탐색 준비 완료 — on_point를 Nozzle 모드로 재활성화")
                self.orient_positive_x("휘발유")

            elif self.mode == "nozzle":
                # 주유건 쪽으로 접근
                wait(2)
                self.gripper_move(600)

        except Exception as e:
            self.get_logger().error(f"❌ 이동 실패: {e}")
        finally:
            self.is_busy = False

    # ─────────────────────────────────────────────────────────
    # 주유 완료
    def finish_fueling(self):
        """주유 완료 처리 (그리퍼 동작 + REST 전송 + 상태 갱신)"""
        self.current_state = "DONE"

        # ② REST 서버로 주유 완료 신호 전송
        try:
            import requests
            payload = {"order_id": getattr(self, "order_id", "UNKNOWN"), "status": "done"}
            url = "http://localhost:8000/fuel/complete"  # 🔧 필요시 서버 IP 변경
            response = requests.post(url, json=payload, timeout=3)
            if response.status_code == 200:
                self.get_logger().info(f"🌐 REST 전송 성공: {response.text}")
            else:
                self.get_logger().warn(f"⚠️ REST 응답 코드: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"❌ REST 전송 실패: {e}")

        # ③ 로그 및 상태 출력
        self.get_logger().info("🏁 주유 프로세스 완료 (FSM: DONE)")

    # ─────────────────────────────────────────────────────────
    def check_crash(self, ori:int):
        from DSR_ROBOT2 import (task_compliance_ctrl, set_desired_force, get_tool_force,
            release_force, release_compliance_ctrl, wait, DR_MV_MOD_REL)
        from DR_common2 import posx
        
        k_d = [500.0, 500.0, 500.0, 200.0, 200.0, 200.0]
        task_compliance_ctrl(k_d)
        # 강성 제어
        if ori == 1:
            f_d = [0.0, -40.0, 0, 0.0, 0.0, 0.0]
            f_dir = [0, -1, 0, 0, 0, 0]

            set_desired_force(f_d, f_dir)
            wait(2.0)

            # 외력감지
            while True:
                force_ext = get_tool_force()
                # c_pos = get_current_posx()
                # x, y, z = c_pos[0]
                if force_ext[1] > 5:
                    release_force()
                    release_compliance_ctrl()

                    break

                # 🔥 ROS 이벤트 처리: 이거 넣으면 음성 출력 가능해짐
                rclpy.spin_once(self, timeout_sec=0.01)

        elif ori == 2:
            f_d = [0.0, 0.0, -40, 0.0, 0.0, 0.0]
            f_dir = [0, 0, 1, 0, 0, 0]

            set_desired_force(f_d, f_dir)
            wait(2.0)

            # 외력감지
            while True:
                force_ext = get_tool_force()
                # c_pos = get_current_posx()
                # x, y, z = c_pos[0]
                if force_ext[2] > 3:
                    release_force()
                    release_compliance_ctrl()

                    self.gripper_move(0)
                    break

    # 주유구를 오픈하기 위해 그리퍼를 회전시키는 함수
    def rotate_grip(self, cnt: int, b_open: bool = True):
        from DSR_ROBOT2 import (amovel, DR_MV_MOD_REL,
            movel, movej, wait)
        from DR_common2 import posx, posj
        count = 0
        open_angle = -120 if b_open else 120

        if b_open:
            while count < cnt :
                self.gripper_move(450)
                
                movej(posj(0, 0, 0, 0, 0, open_angle), v=80, a=80, mod=DR_MV_MOD_REL)
                wait(1.0)
                count = count + 1

                if count < cnt:
                    self.gripper_move(80)
                    movej(posj(0, 0, 0, 0, 0, -open_angle), v=80, a=80, mod=DR_MV_MOD_REL)
                    wait(1.0)

    # ─────────────────────────────────────────────────────────
    def orient_positive_x(self, type:str):
        from DSR_ROBOT2 import get_current_posx, get_current_posj, movej, movel, wait, DR_MV_MOD_REL, DR_MV_MOD_ABS
        from DR_common2 import posx, posj
        # 유종 상태 확인
        # fuel_type = getattr(self, "fuel_type", "").lower()

        # 🔥 Narrator: 노즐 쪽으로 이동
        # self.narrator.narrate("switch_to_nozzle", fuel_type=self.fuel_type or type)

        self.mode = "nozzle"
        self.start_fuel = True
        # 휘발유 → XL / 경유 → XR
        if "gas" in type or "휘발유" in type:
            target_pose = posj(*ORIENT_POSJ_POS_XL)
            # label = "휘발유(+XL)"
        elif "diesel" in type or "경유" in type:
            target_pose = posj(*ORIENT_POSJ_POS_XR)
            # label = "경유(+XR)"
        else:
            # 기본은 XL로 설정
            target_pose = posj(*ORIENT_POSJ_POS_XL)
            # label = "기본(+XL, 유종 미지정)"
        
        movej(target_pose, v=50, a=50, mod=DR_MV_MOD_ABS)
        wait(2)
        
        movel(posx(60, 0, 0, 0, 0, 0), v=60, a=60, mod=DR_MV_MOD_REL)
        wait(1.5)
        self.gripper_move(480)

        movel(posx(-90, 0, 25, 0, 0, 0), v=60, a=60, mod=DR_MV_MOD_REL)
        wait(1.5)

        self.ORIENT_GUN_POS = get_current_posj()

        # 저장했던 주유구 위치로 이동해서 주유건 꽂기
        movej(posj(*self.ORIENT_FUEL_POS), v=80, a=80)
        wait(2)

        movel(posx(0, -50, -100, 0, 0, 0), v=60, a=60, mod=DR_MV_MOD_REL)
        wait(1.5)
        
        self.check_crash(1)
        self.run_fuel_task(3)

        movel(posx(0, 60, 0, 0, 0, 0), v=60, a=60, mod=DR_MV_MOD_REL)
        wait(1.5)

        # 주유 완료 후 다시 주유건 위치로 이동
        movej(posj(*self.ORIENT_GUN_POS), v=80, a=80)
        wait(2)

        movel(posx(90, 0, -15, 0, 0, 0), v=40, a=40, mod=DR_MV_MOD_REL)
        wait(1.5)

        self.gripper_move(0)

        movel(posx(-90, 0, 25, 0, 0, 0), v=40, a=40, mod=DR_MV_MOD_REL)
        wait(1.5)

        movej([0, 0, 90, 0, 90, 0], 80, 80)
        wait(1.0)
        self.gripper_move(700)

        self.check_crash(2)

        movel(posx(0, 0, -30, 0, 0, 0), v=40, a=40, mod=DR_MV_MOD_REL)
        wait(1.5)
        self.gripper_move(450)

        movej(posj(*self.ORIENT_CAP_POS), v=80, a=80)
        wait(2)

        self.rotate_grip(2, False)
        self.gripper_move(0)

        movel(posx(0, 30, 0, 0, 0, 0), v=40, a=40, mod=DR_MV_MOD_REL)
        wait(1.5)

        movej([0, 0, 90, 0, 90, 0], 80, 80)
        wait(1.0)

        # 🔥 Narrator: 노즐 복귀 + 전체 완료
        self.narrator.narrate("return_nozzle")
        self.narrator.narrate("finish")

        # 모든 주유 시퀀스 완료
        self._init_gripper_and_home()

    def orient_negative_y(self):
        self.current_state = "IN_PROGRESS"

        from DSR_ROBOT2 import movej, wait, DR_MV_MOD_ABS
        from DR_common2 import posj

        # 이동 시작 → 감지 중단
        self.is_moving = True
        # 🔥 Narrator
        self.narrator.narrate("orient_negative_y")

        self.get_logger().info("🧭 툴을 -Y(바닥) 방향으로 회전 중…")
        target_pose = posj(*ORIENT_PRESET_POSJ)
        movej(target_pose, v=50, a=50, mod=DR_MV_MOD_ABS)
        wait(2)
        self.get_logger().info("✅ 툴 방향 전환 완료 (-Y)")
        
        self.get_logger().info("그리퍼 초기 위치 오픈")
        self.gripper_move(700)
        wait(1.5)

        # 이동 종료 → 감지 재개
        self.is_moving = False

    def orient_z_down(self):
        from DSR_ROBOT2 import (get_current_posx, movel, wait, DR_MV_MOD_ABS)
        from DR_common2 import posx

        # 이동 시작 → 감지 중단
        self.is_moving = True
        
        c_pos = get_current_posx()
        x, y, z = c_pos[0][0:3]
        target_pos = posx(x, y, z, 0, 180, 0)

        movel(target_pos, v=50, a=50, mod=DR_MV_MOD_ABS)
        wait(3)
        
        # 이동 종료 → 감지 재개
        self.is_moving = False
    
    def run_fuel_task(self, cnt):
        from DSR_ROBOT2 import wait
        try:
            # 🔥 Narrator: 주유 반복 시작
            self.narrator.narrate("fueling")
            for i in range(cnt):
                self.gripper.move(650)
                wait(1.5)
                self.gripper.move(500)
                wait(1.5)

        except Exception as e:
            self.get_logger().error(f"❌ Gripper 반복 동작 중 오류: {e}")  
    # ─────────────────────────────────────────────────────────
    # 멀티모달 TTS
    def say(self, text: str):
        try:
            from gtts import gTTS
            import os

            mp3_path = "/tmp/tts_output.mp3"
            wav_path = "/tmp/tts_output.wav"

            # 1) TTS → mp3 생성
            tts = gTTS(text=text, lang='ko')
            tts.save(mp3_path)

            # 2) mp3 → wav 변환 (ffmpeg 필요)
            os.system(f"ffmpeg -y -i {mp3_path} {wav_path} > /dev/null 2>&1")

            # 3) wav 재생 (시스템에서 확실히 재생됨)
            os.system(f"aplay {wav_path}")

        except Exception as e:
            self.get_logger().error(f"TTS Error: {e}")

    # ─────────────────────────────────────────────────────────
    # 정리
    def terminate_gripper(self):
        try:
            if hasattr(self, 'gripper') and self.gripper:
                self.gripper.shutdown()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)

    # DSR 초기 노드 선언(권장 순서): 별도 노드 등록
    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    node = MotionController()
    node.get_logger().info("✅ Hand–Eye 멀티모드 버전 실행 중.('fuel_cap'|'nozzle'|'idle')로 모드 전환 가능.")
    # node.orient_negative_y()      # -Y 방향 전환
    # node.search_for_object()      # 바로 탐색 시작

    # node.orient_positive_x("휘발유")      # -X 방향 전환
    # node.search_for_object()      # 바로 탐색 시작

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 MotionController stopped.")
        node.terminate_gripper()
    finally:
        node.terminate_gripper()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
