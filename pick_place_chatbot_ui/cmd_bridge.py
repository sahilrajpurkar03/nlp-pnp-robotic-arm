import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
import uvicorn
import os

from llm_mapper import llm_extract_label, VALID_LABELS

# ---------- ROS2 Node ----------
class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.pub = self.create_publisher(String, '/target_class_cmd', 10)

    def publish_label(self, label: str):
        msg = String()
        msg.data = label
        self.pub.publish(msg)
        self.get_logger().info(f"Published /target_class_cmd: {label}")

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
    label = llm_extract_label(req.text)
    if not label:
        return {"ok": False, "message": "Could not map to a known object.", "labels": VALID_LABELS}
    assert ros_node is not None
    ros_node.publish_label(label)
    return {"ok": True, "label": label}

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
