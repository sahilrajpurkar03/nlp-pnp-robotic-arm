import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
import uvicorn
import os

from llm_mapper import llm_chat_or_pick, VALID_LABELS

# ---------- ROS2 Node ----------
class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.pub = self.create_publisher(String, '/target_class_cmd', 10)

        # Subscribe to robot pick/place status
        self.status_sub = self.create_subscription(
            String,
            '/pick_place_status',
            self.status_callback,
            10
        )
        self.current_status = "IDLE"
        self.last_pick = None
        self.websocket_clients: set[WebSocket] = set()

        # Track latest detected objects
        self.latest_detections: set[str] = set()

        # Subscribe to YOLO detections
        try:
            from yolov8_msgs.msg import Yolov8Inference
            self.yolo_sub = self.create_subscription(
                Yolov8Inference,
                '/Yolov8_Inference',
                self.yolo_callback,
                10
            )
        except ImportError:
            self.get_logger().warn("Yolov8Inference message type not found. Detection check disabled.")

    def publish_label(self, label: str, box: str | None = None):
        msg = String()
        msg.data = f"{label},{box}" if box else label
        self.pub.publish(msg)
        self.get_logger().info(f"Published /target_class_cmd: {msg.data}")

    def _broadcast_json(self, payload: dict):
        """Send JSON to connected websockets (best-effort)."""
        for ws in list(self.websocket_clients):
            try:
                import asyncio
                asyncio.run(ws.send_json(payload))
            except Exception as e:
                self.get_logger().warn(f"WebSocket send failed: {e}")
                self.websocket_clients.discard(ws)

    def status_callback(self, msg: String):
        new_status = msg.data.strip().upper()
        if self.current_status != new_status:
            self.current_status = new_status
            self.get_logger().info(f"Robot status: {self.current_status}")

            # When robot goes back to IDLE, confirm success
            if self.current_status == "IDLE" and self.last_pick:
                if "active" in self.last_pick:
                    label = self.last_pick["active"].replace("_", " ")
                    box = self.last_pick["box"]
                    success_msg = f"{label} dropped in box {box} ✅"
                    self.get_logger().info(success_msg)

                    # Push success message to all websocket clients
                    self._broadcast_json({"reply": success_msg})

                    # Continue queue if more items left
                    if self.last_pick["queue"]:
                        next_label = self.last_pick["queue"].pop(0)
                        # Publish next item to ROS
                        self.publish_label(next_label, box)
                        self.last_pick["active"] = next_label

                        # Send the picking-in-progress message for next item via WS
                        picking_msg = f"Picking the {next_label.replace('_',' ')} ⏳ in progress..."
                        self._broadcast_json({"reply": picking_msg})
                    else:
                        # All done
                        self.last_pick = None

    def yolo_callback(self, msg):
        """Update latest detected objects from YOLO."""
        try:
            self.latest_detections = {det.class_name for det in msg.yolov8_inference}
            self.get_logger().debug(f"Latest detected objects: {self.latest_detections}")
        except Exception as e:
            self.get_logger().warn(f"Error reading YOLO detections: {e}")

# ---------- FastAPI ----------
app = FastAPI(title="Robot Arm Command Chatbot Bridge")
ros_node: CommandPublisher | None = None

class ChatRequest(BaseModel):
    text: str

# Serve HTML frontend
@app.get("/", response_class=HTMLResponse)
def home():
    index_path = os.path.join(os.path.dirname(__file__), "index.html")
    with open(index_path, "r") as f:
        return HTMLResponse(f.read())

@app.get("/labels")
def labels():
    return {"labels": VALID_LABELS}

@app.post("/chat")
def chat(req: ChatRequest):
    if ros_node is None:
        return {"ok": True, "reply": "Robot node not ready."}

    # Pass current detections to LLM handler
    data = llm_chat_or_pick(req.text, visible_objects=ros_node.latest_detections)

    if data.get("type") in ["ask_box", "pick"]:
        label = data["label"] if "label" in data else None

        # Direct pick if box already specified
        if data.get("type") == "pick":
            ros_node.publish_label(label, data.get("box"))
            ros_node.last_pick = {"label": label, "box": data.get("box")}
        elif data.get("type") == "ask_box":
            # store queue awaiting box; handler will set last_pick when box arrives
            ros_node.last_pick = {"queue": data["labels"], "awaiting_box": True}

        return {"ok": True, "reply": data["reply"]}

    # pick_queue: labels + box provided (multiple items)
    if data.get("type") == "pick_queue":
        # set queue and immediately dispatch the first item, return picking spinner for first item
        ros_node.last_pick = {"queue": data["labels"], "box": data["box"]}
        first = ros_node.last_pick["queue"].pop(0)
        ros_node.publish_label(first, data["box"])
        ros_node.last_pick["active"] = first
        # Return spinner message for the first item (HTTP response)
        return {"ok": True, "reply": f"Picking the {first.replace('_',' ')} ⏳ in progress..."}

    return {"ok": True, "reply": data["reply"]}

# ---------- WebSocket for live updates ----------
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    if ros_node:
        ros_node.websocket_clients.add(websocket)
    try:
        while True:
            await websocket.receive_text()  # keep alive / noop
    except WebSocketDisconnect:
        if ros_node:
            ros_node.websocket_clients.discard(websocket)

# ---------- FastAPI Thread ----------
def start_fastapi():
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")

# ---------- Main ----------
def main():
    global ros_node
    rclpy.init()
    ros_node = CommandPublisher()

    # Run FastAPI in a separate thread
    api_thread = threading.Thread(target=start_fastapi, daemon=True)
    api_thread.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
