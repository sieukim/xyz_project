from fastapi import FastAPI, BackgroundTasks, HTTPException
from pydantic import BaseModel
from typing import Dict, Any
import json
import time
import threading
import queue
import subprocess
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI(title="SmartFuel ROS2 Bridge")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 👈 모든 도메인에서의 요청을 허용
    allow_credentials=True,
    allow_methods=["*"],  # 👈 OPTIONS, GET, POST 등 모든 메소드 허용
    allow_headers=["*"],  # 👈 모든 헤더 허용
)

# 간단한 인메모리 상태 저장소 (프로토타입용)
orders: Dict[str, Dict[str, Any]] = {}


class StartRequest(BaseModel):
    orderId: str
    fuelType: str
    amount: int
    source: str = "mobile_app"


def _simulate_progress(order_id: str):
    # 프로토타입: 0% -> 100% 까지 단계적으로 증가시키는 시뮬레이션
    orders[order_id]["status"] = "in_progress"
    for p in range(0, 101, 5):  # 10단계 -> 20단계로 변경
        orders[order_id]["progress"] = p
        # 실제 구현에서는 ROS2의 상태 토픽을 구독하여 진행도를 업데이트해야 합니다.
        time.sleep(1)  # 1초씩 20번 = 총 20초
    orders[order_id]["status"] = "completed"
    orders[order_id]["progress"] = 100


def _publish_to_ros2(payload: dict):
    # 기존 함수는 더 이상 사용하지 않습니다. 퍼블리셔 스레드 또는 CLI 폴백을 사용하세요.
    raise RuntimeError("_publish_to_ros2 is deprecated; use enqueue_publish instead")


# --- Persistent ROS2 publisher implementation --------------------------------
# This creates a background thread that initializes rclpy and a Node once,
# then consumes payloads from a thread-safe queue and publishes them.
ros_queue: "queue.Queue[dict]" = queue.Queue()
ros_thread: threading.Thread | None = None
ros_running = threading.Event()
use_rclpy = False


def _ros_publisher_loop():
    global use_rclpy
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        rclpy.init()
        node = Node("smartfuel_persistent_bridge")
        pub = node.create_publisher(String, "start_fuel", 10)
        use_rclpy = True
        print("rclpy available - persistent ROS2 publisher started")

        # loop until stopped
        while ros_running.is_set():
            try:
                payload = ros_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                msg = String()
                msg.data = json.dumps(payload, ensure_ascii=False)
                pub.publish(msg)
                print(f"Published payload to /start_fuel via rclpy: {payload.get('orderId')}")
            except Exception as e:
                print(f"Failed to publish via rclpy: {e}")

        # shutdown
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        print("rclpy publisher stopped")
    except Exception as e:
        # rclpy unavailable - we'll fall back to CLI on each request
        use_rclpy = False
        print(f"rclpy not available, will use ros2 CLI fallback: {e}")


def start_ros_thread():
    global ros_thread
    if ros_thread is not None and ros_thread.is_alive():
        return
    ros_running.set()
    ros_thread = threading.Thread(target=_ros_publisher_loop, daemon=True)
    ros_thread.start()


def stop_ros_thread():
    ros_running.clear()
    if ros_thread is not None:
        ros_thread.join(timeout=2)


def enqueue_publish(payload: dict) -> bool:
    """Enqueue payload for persistent rclpy publisher, or fallback to ros2 CLI.

    Returns True if publish requested/succeeded, False otherwise.
    """
    if use_rclpy:
        try:
            ros_queue.put_nowait(payload)
            return True
        except Exception as e:
            print(f"Failed to enqueue payload for rclpy publisher: {e}")
            return False

    # Fallback: use ros2 CLI
    try:
        data_str = json.dumps(payload, ensure_ascii=False)
        cmd = [
            "ros2",
            "topic",
            "pub",
            "/start_fuel",
            "std_msgs/msg/String",
            f"{{data: '{data_str}'}}",
            "--once",
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        if proc.returncode == 0:
            print("Published to ROS2 via ros2 CLI (fallback)")
            return True
        else:
            print(f"ros2 CLI publish failed: {proc.returncode} {proc.stderr}")
            return False
    except Exception as e:
        print(f"ros2 CLI fallback failed: {e}")
        return False

# Start ROS publisher thread at startup
@app.on_event("startup")
def _on_startup():
    start_ros_thread()


@app.on_event("shutdown")
def _on_shutdown():
    stop_ros_thread()

# ---------------------------------------------------------------------------


def background_start(order_id: str, payload: dict):
    # Persistent publisher에 요청을 전달 (또는 CLI 폴백)
    ok = enqueue_publish(payload)

    # 상태 업데이트
    orders[order_id]["status"] = "accepted" if ok else "accepted_offline"
    orders[order_id]["progress"] = 0

    # 실제 ROS2 노드가 별도의 상태 토픽을 publish한다면
    # 여기서 구독해서 상태를 업데이트하도록 구현하세요.
    # 우선 시뮬레이션으로 진행 상태를 업데이트합니다.
    _simulate_progress(order_id)


@app.post("/start_fuel")
async def start_fuel(req: StartRequest, background_tasks: BackgroundTasks):
    if not req.orderId:
        raise HTTPException(status_code=400, detail="orderId required")

    if req.orderId in orders:
        raise HTTPException(status_code=409, detail="orderId already exists")

    orders[req.orderId] = {
        "orderId": req.orderId,
        "fuelType": req.fuelType,
        "amount": req.amount,
        "source": req.source,
        "status": "pending",
        "progress": 0,
        "created_at": time.time(),
    }

    # 백그라운드에서 ROS2 발행 + 진행 시뮬레이션
    background_tasks.add_task(background_start, req.orderId, req.dict())

    return {"orderId": req.orderId, "status": "accepted"}


@app.get("/status/{order_id}")
async def get_status(order_id: str):
    if order_id not in orders:
        raise HTTPException(status_code=404, detail="order not found")
    return orders[order_id]
