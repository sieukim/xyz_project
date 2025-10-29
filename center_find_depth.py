import cv2
import numpy as np
from ultralytics import YOLO

# 설정
MODEL_PATH = "/home/deepet/Desktop/yellowsimpson/github/yolo_dectect_traing/runs/detect/train15/weights/best.pt" # 파일 경로 설정

# RealSense 사용 여부 (True 권장)
USE_REALSENSE = True

# 프레임 크기 (RealSense 컬러 스트림 크기)
FRAME_WIDTH, FRAME_HEIGHT = 1280, 720

CONFIDENCE_THRESHOLD = 0.7
CENTER_TOLERANCE_PX = 10  # 프레임 중심 근접 판단용

# 깊이 후처리 설정
DEPTH_WINDOW = 5          # 중심 주변 N×N 윈도우에서 중앙값 사용
MAX_VALID_DEPTH_M = 8.0   # (필요시) 최대 유효 깊이(m) 제한

# RealSense 초기화
rs_pipeline = None
rs_align = None
rs_depth_scale = None

def init_realsense():
    """
    RealSense 파이프라인 시작 + 컬러 기준 정렬 준비
    return: (pipeline, align, depth_scale)
    """
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    # 컬러/깊이 모두 enable
    config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 깊이는 640x480 권장

    profile = pipeline.start(config)

    # 깊이 스케일(단위 변환: 깊이값 * scale = meters)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()  # 보통 0.001 (mm→m)

    # 컬러 프레임에(depth→color) 정렬
    align = rs.align(rs.stream.color)

    return pipeline, align, depth_scale

# 탐지 + 중심 시각화 함수
def annotate_with_detection(frame, yolo_result, depth_image=None, depth_scale=0.001):
    """
    frame: BGR 컬러 프레임
    yolo_result: Ultralytics 결과 객체 (results[0])
    depth_image: 정렬된 깊이 프레임 (단위: depth 단위값, z16), frame과 동일 해상도여야 함
    depth_scale: 깊이 스케일(곱하면 미터)
    return: (annotated_frame, info_dict)
    """
    h, w = frame.shape[:2]
    target_x, target_y = w // 2, h // 2  # 프레임 중심
    annotated = yolo_result.plot()

    # 프레임 중심 마커(흰색 X 표시)
    cv2.drawMarker(
        annotated, (target_x, target_y), (255, 255, 255),
        markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2
    )

    info = {
        "detected": False,
        "center_x": None,
        "center_y": None,
        "dx_pixels": None,
        "dy_pixels": None,
        "centered": False,
        "conf": None,
        "distance_m": None,  # 추가: 거리(m)
    }

    boxes = getattr(yolo_result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        cv2.putText(annotated, "No detection", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (50, 50, 255), 2)
        return annotated, info

    # 신뢰도 높은 박스 하나 선택
    best_idx, best_conf = None, -1.0
    for i in range(len(boxes)):
        conf = float(boxes.conf[i].item()) if hasattr(boxes, "conf") else 0.0
        if conf >= CONFIDENCE_THRESHOLD and conf > best_conf:
            best_idx, best_conf = i, conf

    if best_idx is None:
        cv2.putText(annotated, "Low-confidence detections only", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 140, 255), 2)
        return annotated, info

    # 바운딩 박스 중심
    x1, y1, x2, y2 = map(int, boxes.xyxy[best_idx].tolist())
    x_center = (x1 + x2) // 2
    y_center = (y1 + y2) // 2

    # 빨간 점(객체 중심)
    cv2.circle(annotated, (x_center, y_center), 6, (0, 0, 255), -1)

    # 프레임 중심과의 오차(픽셀)
    dx_pixels = (target_x - x_center)
    dy_pixels = (target_y - y_center)

    # ---- 깊이 읽기 (정렬된 depth_image가 있는 경우에만) ----
    distance_m = None
    if depth_image is not None:
        # 유효 범위 내 인덱스 보정
        cx = int(np.clip(x_center, 0, depth_image.shape[1]-1))
        cy = int(np.clip(y_center, 0, depth_image.shape[0]-1))

        # 노이즈 완화: 중심 주변 윈도우(DEPTH_WINDOW x DEPTH_WINDOW) 중앙값
        half = max(1, DEPTH_WINDOW // 2)
        x0, x1w = max(0, cx - half), min(depth_image.shape[1], cx + half + 1)
        y0, y1w = max(0, cy - half), min(depth_image.shape[0], cy + half + 1)

        patch = depth_image[y0:y1w, x0:x1w].astype(np.float32)

        # 0(미계측) 제거
        patch = patch[patch > 0]
        if patch.size > 0:
            median_depth_unit = float(np.median(patch))
            distance_m = median_depth_unit * depth_scale
            # (선택) 말도 안되는 초과값 클리핑
            if distance_m > MAX_VALID_DEPTH_M:
                distance_m = None

    # 텍스트 오버레이
    cv2.putText(annotated, f"center=({x_center},{y_center}) conf={best_conf:.2f}",
                (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 255, 200), 2)
    cv2.putText(annotated, f"delta_px=({dx_pixels},{dy_pixels})",
                (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 255, 200), 2)

    if distance_m is not None:
        cv2.putText(annotated, f"distance={distance_m:.3f} m",
                    (20, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    else:
        cv2.putText(annotated, f"distance=---",
                    (20, 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2)

    centered = (abs(dx_pixels) < CENTER_TOLERANCE_PX and abs(dy_pixels) < CENTER_TOLERANCE_PX)
    if centered:
        cv2.putText(annotated, "Centered!", (20, 210),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 2)

    info.update({
        "detected": True,
        "center_x": x_center,
        "center_y": y_center,
        "dx_pixels": dx_pixels,
        "dy_pixels": dy_pixels,
        "centered": centered,
        "conf": best_conf,
        "distance_m": distance_m,
    })
    return annotated, info


def main():
    model = YOLO(MODEL_PATH)

    # --- 입력 소스 준비 ---
    color_frame = None
    depth_frame = None
    global rs_pipeline, rs_align, rs_depth_scale

    if USE_REALSENSE:
        try:
            rs_pipeline, rs_align, rs_depth_scale = init_realsense()
        except Exception as e:
            print(f"⚠️ RealSense 초기화 실패: {e}")
            return
        print(f"[INFO] RealSense depth_scale: {rs_depth_scale}")
    else:
        # (대안) 일반 UVC 카메라만 사용할 경우(깊이 X)
        cap = cv2.VideoCapture(0)  # 필요 시 장치 경로 변경
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        if not cap.isOpened():
            print("⚠️ 카메라를 열 수 없습니다.")
            return

    WINDOW_NAME = "YOLO + Depth (Aligned)"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while True:
            # ----- 프레임 획득 -----
            if USE_REALSENSE:
                import pyrealsense2 as rs
                frames = rs_pipeline.wait_for_frames()
                # 깊이를 컬러에 정렬
                aligned_frames = rs_align.process(frames)
                depth = aligned_frames.get_depth_frame()
                color = aligned_frames.get_color_frame()
                if not depth or not color:
                    print("⚠️ 프레임 획득 실패(RealSense).")
                    continue

                depth_frame = np.asanyarray(depth.get_data())      # z16
                color_frame = np.asanyarray(color.get_data())      # BGR8
                frame = color_frame
            else:
                ret, frame = cap.read()
                if not ret:
                    print("⚠️ 카메라에서 프레임을 읽을 수 없습니다.")
                    break
                depth_frame = None  # 깊이 없음

            # ----- YOLO 추론 -----
            results = model.predict(
                source=frame,
                device="cpu",
                imgsz=640,
                conf=0.4,
                verbose=False
            )
            r0 = results[0]

            # ----- 주석/거리 산출 -----
            annotated, info = annotate_with_detection(
                frame, r0,
                depth_image=depth_frame if USE_REALSENSE else None,
                depth_scale=rs_depth_scale if USE_REALSENSE else 0.001
            )

            # 콘솔 로그
            if info["detected"]:
                d_str = f"{info['distance_m']:.3f}m" if info["distance_m"] is not None else "---"
                print(
                    f"[DET] center=({info['center_x']},{info['center_y']}) "
                    f"delta=({info['dx_pixels']},{info['dy_pixels']}) "
                    f"conf={info['conf']:.2f} distance={d_str} centered={info['centered']}"
                )

            cv2.imshow(WINDOW_NAME, annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[EXIT] 사용자 종료")
                break

    finally:
        if USE_REALSENSE and rs_pipeline is not None:
            rs_pipeline.stop()
        else:
            try:
                cap.release()
            except:
                pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
