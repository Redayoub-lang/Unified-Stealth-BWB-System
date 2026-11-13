import asyncio
import base64
import json
from contextlib import asynccontextmanager

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

from camera_vision_system import UAVVisionSystem
from stealth_autopilot import StealthAutopilot
from stealth_bwb_physics import StealthBWBPhysics

# تهيئة مكونات النظام الأساسية
DT = 0.03  # معدل التحديث 33.3 Hz
uav = StealthBWBPhysics(dt=DT)
vision = UAVVisionSystem()
autopilot = StealthAutopilot(stealth_mode=True)

flight_mode = "NAV_AUTO"
latest_payload_json = ""


class ConnectionManager:
    """إدارة اتصالات WebSocket المتعددة (لوحة القيادة + جسر ROS 2) بأمان."""

    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        if not self.active_connections:
            return
        dead_connections = []
        for connection in list(self.active_connections):
            try:
                await connection.send_text(message)
            except Exception:
                dead_connections.append(connection)
        for dead in dead_connections:
            self.disconnect(dead)


manager = ConnectionManager()


async def flight_control_loop():
    """
    حلقة التحكم الرئيسية المستقلة (33 Hz):
    تُحدّث الفيزياء، الطيار الآلي، والرؤية الحاسوبية، ثم تبث الحزمة لجميع العملاء.
    """
    global flight_mode, latest_payload_json

    while True:
        loop_start = asyncio.get_event_loop().time()

        # 1. حساب أوامر التحكم بناءً على وضع الطيران الحالي
        if flight_mode in ("NAV_AUTO", "RTL"):
            telemetry_preview = {
                "position": uav.pos.tolist(),
                "euler": np.degrees(uav.euler).tolist(),
                "airspeed": uav.airspeed,
                "battery_pct": uav.battery_pct
            }
            e_left, e_right, t_cmd = autopilot.compute_control_surface_commands(telemetry_preview, dt=DT)

            # تحديث وضع العرض تلقائياً إذا فعّل الطيار الآلي العودة الاضطرارية للبطارية
            if autopilot.rtl_triggered and flight_mode != "RTL":
                flight_mode = "RTL"

            telemetry_data = uav.step(
                delta_elevon_left=e_left,
                delta_elevon_right=e_right,
                delta_thrust=t_cmd
            )

        elif flight_mode == "TAKEOFF":
            # دفع مرتفع مع زاوية صعود تدريجية حتى الوصول لارتفاع آمن (50 متر)
            telemetry_data = uav.step(delta_elevon_left=0.14, delta_elevon_right=0.14, delta_thrust=0.85)
            if telemetry_data["altitude"] >= 50.0:
                flight_mode = "NAV_AUTO"

        elif flight_mode == "GLIDE":
            # انزلاق هوائي بدون دفع (إيقاف المحرك)
            telemetry_data = uav.step(delta_elevon_left=0.02, delta_elevon_right=0.02, delta_thrust=0.0)

        else:  # وضع الإبحار الثابت CRUISE / HOLD
            telemetry_data = uav.step(delta_elevon_left=0.0, delta_elevon_right=0.0, delta_thrust=0.45)

        # 2. معالجة إطار الكاميرا وضغطه بجودة متوازنة لتقليل تأخير البث (Low Latency)
        frame = vision.process_frame(telemetry_data)
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')

        # 3. تجهيز حزمة البيانات الموحدة (متوافقة 100% مع dashboard.html و ROS 2)
        target_wp = (
            autopilot.waypoints[autopilot.current_wp_index]
            if autopilot.waypoints and autopilot.current_wp_index < len(autopilot.waypoints)
            else None
        )
        if flight_mode == "RTL":
            target_wp = autopilot.home_position

        payload = {
            "telemetry": telemetry_data,
            "mode": flight_mode,
            "waypoints": autopilot.waypoints,
            "target_wp": target_wp,
            "frame": jpg_as_text
        }
        latest_payload_json = json.dumps(payload)

        # 4. بث البيانات لجميع الواجهات المتصلة
        await manager.broadcast(latest_payload_json)

        # الحفاظ على دقة التوقيت 33 Hz
        elapsed = asyncio.get_event_loop().time() - loop_start
        await asyncio.sleep(max(0.001, DT - elapsed))


@asynccontextmanager
async def lifespan(app: FastAPI):
    """إدارة بدء وإيقاف حلقة الطيران وتحرير الكاميرا بأمان عند إغلاق السيرفر."""
    task = asyncio.create_task(flight_control_loop())
    yield
    task.cancel()
    vision.release()


app = FastAPI(title="Unified Stealth BWB C2 Server", lifespan=lifespan)


@app.get("/")
async def get_dashboard():
    with open("dashboard.html", "r", encoding="utf-8") as f:
        return HTMLResponse(content=f.read())


@app.websocket("/ws/telemetry")
async def websocket_endpoint(websocket: WebSocket):
    global flight_mode, uav, autopilot
    await manager.connect(websocket)
    try:
        while True:
            raw_msg = await websocket.receive_text()
            try:
                msg = json.loads(raw_msg)
            except json.JSONDecodeError:
                continue

            if "command" in msg:
                cmd = str(msg["command"]).upper()
                if cmd == "RTL":
                    autopilot.trigger_rtl()
                    flight_mode = "RTL"
                elif cmd == "RESET":
                    uav = StealthBWBPhysics(dt=DT)
                    autopilot = StealthAutopilot(stealth_mode=autopilot.stealth_mode)
                    flight_mode = "NAV_AUTO"
                else:
                    autopilot.rtl_triggered = False
                    flight_mode = cmd

            if "stealth_mode" in msg:
                autopilot.stealth_mode = bool(msg["stealth_mode"])

            if "add_wp" in msg and isinstance(msg["add_wp"], list) and len(msg["add_wp"]) == 3:
                autopilot.waypoints.append([float(v) for v in msg["add_wp"]])

            if "clear_wps" in msg:
                autopilot.waypoints = []
                autopilot.current_wp_index = 0

    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception:
        manager.disconnect(websocket)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)