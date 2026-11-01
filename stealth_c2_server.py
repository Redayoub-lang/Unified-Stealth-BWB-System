import asyncio
import json
import cv2
import base64
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn

from stealth_bwb_physics import StealthBWBPhysics
from camera_vision_system import UAVVisionSystem
from stealth_autopilot import StealthAutopilot

app = FastAPI()
uav = StealthBWBPhysics(dt=0.03)
vision = UAVVisionSystem()
autopilot = StealthAutopilot()

flight_mode = "NAV_AUTO"


@app.get("/")
async def get_dashboard():
    # كود HTML السابق مع إضافة وضع NAV_AUTO والنقاط
    with open("dashboard.html", "r", encoding="utf-8") as f:
        html_content = f.read()
    return HTMLResponse(content=html_content)


@app.websocket("/ws/telemetry")
async def websocket_endpoint(websocket: WebSocket):
    global flight_mode
    await websocket.accept()
    try:
        while True:
            try:
                raw_msg = await asyncio.wait_for(websocket.receive_text(), timeout=0.001)
                msg = json.loads(raw_msg)
                if "command" in msg:
                    flight_mode = msg["command"]
                if "waypoints" in msg:
                    autopilot.set_waypoints(msg["waypoints"])
            except asyncio.TimeoutError:
                pass

            # تفعيل التحكم
            if flight_mode == "NAV_AUTO":
                telemetry_preview = {
                    "position": uav.pos.tolist(),
                    "euler": np.degrees(uav.euler).tolist()
                }
                e_left, e_right, t_cmd = autopilot.compute_control_surface_commands(telemetry_preview)
                telemetry_data = uav.step(delta_elevon_left=e_left, delta_elevon_right=e_right, delta_thrust=t_cmd)
            else:
                t_in = 0.85 if flight_mode == "TAKEOFF" else (0.0 if flight_mode == "GLIDE" else 0.45)
                telemetry_data = uav.step(delta_thrust=t_in)

            frame = vision.process_frame(telemetry_data)
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')

            payload = {
                "telemetry": telemetry_data,
                "mode": flight_mode,
                "target_wp": autopilot.waypoints[autopilot.current_wp_index] if autopilot.waypoints else None,
                "frame": jpg_as_text
            }

            await websocket.send_text(json.dumps(payload))
            await asyncio.sleep(0.03)
    except WebSocketDisconnect:
        pass

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)