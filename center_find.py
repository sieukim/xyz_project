import cv2
from ultralytics import YOLO

# =========================
# 설정
# =========================
MODEL_PATH = "/home/deepet/Desktop/yellowsimpson/github/xyz_1st_project/runs/detect/train/weights/best.pt"
CAMERA_DEV = "/dev/video8" #"/dev/video7"
FRAME_WIDTH, FRAME_HEIGHT = 1280, 720
CONFIDENCE_THRESHOLD = 0.7
CENTER_TOLERANCE_PX = 10  # 프레임 중심 근접 판단용

# =========================
# 탐지 + 중심 시각화 함수
# =========================
def annotate_with_detection(frame, yolo_result):
    """
    frame: BGR 프레임
    yolo_result: Ultralytics 결과 객체 (results[0])
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
    }

    boxes = getattr(yolo_result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        # 감지 없음
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
        # 신뢰도 기준 미달
        cv2.putText(annotated, "Low-confidence detections only", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 140, 255), 2)
        return annotated, info

    # 바운딩 박스 중심 계산
    x1, y1, x2, y2 = map(int, boxes.xyxy[best_idx].tolist())
    x_center = (x1 + x2) // 2
    y_center = (y1 + y2) // 2

    # 빨간 점(객체 중심)
    cv2.circle(annotated, (x_center, y_center), 6, (0, 0, 255), -1)

    # 프레임 중심과의 오차(픽셀)
    dx_pixels = (target_x - x_center)
    dy_pixels = (target_y - y_center)

    # 텍스트 오버레이
    cv2.putText(annotated, f"center=({x_center},{y_center}) conf={best_conf:.2f}",
                (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 255, 200), 2)
    cv2.putText(annotated, f"delta_px=({dx_pixels},{dy_pixels})",
                (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 255, 200), 2)

    centered = (abs(dx_pixels) < CENTER_TOLERANCE_PX and abs(dy_pixels) < CENTER_TOLERANCE_PX)
    if centered:
        cv2.putText(annotated, "Centered!", (20, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 2)

    info.update({
        "detected": True,
        "center_x": x_center,
        "center_y": y_center,
        "dx_pixels": dx_pixels,
        "dy_pixels": dy_pixels,
        "centered": centered,
        "conf": best_conf,
    })
    return annotated, info

def main():
    model = YOLO(MODEL_PATH)

    cap = cv2.VideoCapture(CAMERA_DEV)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        print(f"⚠️ 카메라를 열 수 없습니다: {CAMERA_DEV}")
        return

    WINDOW_NAME = "YOLO Detection View (No Robot Control)"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("⚠️ 카메라에서 프레임을 읽을 수 없습니다.")
                break

            # YOLO 추론
            results = model.predict(
                source=frame,
                device="cpu",
                imgsz=640,
                conf=0.4,
                verbose=False
            )
            r0 = results[0]

            annotated, info = annotate_with_detection(frame, r0)

            # (선택) 콘솔 로그
            if info["detected"]:
                print(f"[DET] center=({info['center_x']},{info['center_y']}) "
                      f"delta=({info['dx_pixels']},{info['dy_pixels']}) "
                      f"conf={info['conf']:.2f} centered={info['centered']}")

            cv2.imshow(WINDOW_NAME, annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[EXIT] 사용자 종료")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
