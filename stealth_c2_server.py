import asyncio
import json
import cv2
import base64
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn
from stealth_bwb_physics import StealthBWBPhysics
from camera_vision_system import UAVVisionSystem

app = FastAPI()
uav = StealthBWBPhysics(dt=0.03)
vision = UAVVisionSystem()

flight_mode = "CRUISE"


@app.get("/")
async def get_dashboard():
    html_content = """
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <title>Stealth BWB UAV Command & Control</title>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
        <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
        <style>
            body { margin: 0; background: #070a12; color: #f8fafc; font-family: monospace; overflow: hidden; }
            #hud { position: absolute; top: 15px; left: 15px; background: rgba(11, 15, 25, 0.92); padding: 18px; border-radius: 12px; border: 1px solid #0284c7; width: 320px; z-index: 10; }
            #vision-panel { position: absolute; bottom: 15px; right: 15px; border: 1px solid #0284c7; border-radius: 8px; overflow: hidden; width: 320px; }
            #vision-feed { width: 100%; height: 240px; background: #000; }
            h2 { margin: 0 0 10px 0; color: #38bdf8; font-size: 14px; }
            .metric { margin-bottom: 6px; font-size: 12px; display: flex; justify-content: space-between; }
            .val { color: #4ade80; font-weight: bold; }
            .btn-group { margin-top: 12px; display: flex; gap: 6px; }
            button { flex: 1; padding: 6px; background: #0369a1; color: white; border: none; border-radius: 4px; cursor: pointer; font-family: monospace; font-weight: bold; }
        </style>
    </head>
    <body>
        <div id="hud">
            <h2>⚡ STEALTH BWB COMMAND CENTER</h2>
            <div class="metric">Status: <span class="val" id="net-status">CONNECTING...</span></div>
            <div class="metric">Flight Mode: <span class="val" id="mode">CRUISE</span></div>
            <div class="metric">Airspeed: <span class="val" id="speed">0.0</span> m/s</div>
            <div class="metric">Altitude: <span class="val" id="alt">0.0</span> m</div>
            <div class="metric">L/D Ratio: <span class="val" id="ld">0.0</span></div>
            <div class="metric">RCS: <span class="val" style="color:#f59e0b;" id="rcs">0.0</span> dBsm</div>
            <div class="metric">Battery: <span class="val" id="bat">100</span>%</div>
            
            <div class="btn-group">
                <button onclick="sendCmd('TAKEOFF')">TAKEOFF</button>
                <button onclick="sendCmd('CRUISE')">CRUISE</button>
                <button onclick="sendCmd('GLIDE')">GLIDE</button>
            </div>
        </div>

        <div id="vision-panel">
            <img id="vision-feed" alt="Optical Vision Feed"/>
        </div>

        <script>
            let scene = new THREE.Scene();
            scene.background = new THREE.Color(0x070a12);

            let camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 1000);
            camera.position.set(0, -6, 3.5);

            let renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
            renderer.setSize(window.innerWidth, window.innerHeight);
            document.body.appendChild(renderer.domElement);

            let controls = new THREE.OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;

            let grid = new THREE.GridHelper(50, 50, 0x0284c7, 0x1e293b);
            grid.rotation.x = Math.PI / 2;
            scene.add(grid);

            scene.add(new THREE.DirectionalLight(0xffffff, 1.2));
            scene.add(new THREE.AmbientLight(0x334155));

            function createStealthBWB() {
                let group = new THREE.Group();
                let mat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.8, roughness: 0.2 });
                let edgeMat = new THREE.MeshStandardMaterial({ color: 0x0284c7 });

                let bodyGeo = new THREE.ConeGeometry(0.45, 1.6, 4);
                let body = new THREE.Mesh(bodyGeo, mat);
                body.rotation.x = -Math.PI / 2;
                body.scale.set(1.2, 1.0, 0.35);
                group.add(body);

                let wingShape = new THREE.Shape();
                wingShape.moveTo(0, 0.8);
                wingShape.lineTo(1.4, -0.4);
                wingShape.lineTo(1.3, -0.7);
                wingShape.lineTo(0, -0.4);
                wingShape.lineTo(-1.3, -0.7);
                wingShape.lineTo(-1.4, -0.4);
                wingShape.closePath();

                let wingGeo = new THREE.ExtrudeGeometry(wingShape, { depth: 0.04, bevelEnabled: true, bevelThickness: 0.02 });
                let wing = new THREE.Mesh(wingGeo, edgeMat);
                wing.rotation.x = Math.PI / 2;
                wing.position.z = 0.02;
                group.add(wing);

                return group;
            }

            let uavMesh = createStealthBWB();
            scene.add(uavMesh);

            let ws = new WebSocket(`ws://${location.host}/ws/telemetry`);
            ws.onopen = () => { document.getElementById('net-status').innerText = 'LIVE ONLINE'; };

            ws.onmessage = (event) => {
                let data = JSON.parse(event.data);
                document.getElementById('speed').innerText = data.telemetry.airspeed.toFixed(1);
                document.getElementById('alt').innerText = (-data.telemetry.position[2]).toFixed(1);
                document.getElementById('ld').innerText = data.telemetry.lift_to_drag.toFixed(1);
                document.getElementById('rcs').innerText = data.telemetry.rcs_db.toFixed(2);
                document.getElementById('bat').innerText = data.telemetry.battery_percent.toFixed(1);
                document.getElementById('mode').innerText = data.mode;

                uavMesh.position.set(data.telemetry.position[0], data.telemetry.position[1], -data.telemetry.position[2]);
                uavMesh.rotation.set(
                    THREE.MathUtils.degToRad(data.telemetry.euler[0]),
                    THREE.MathUtils.degToRad(data.telemetry.euler[1]),
                    THREE.MathUtils.degToRad(data.telemetry.euler[2]),
                    'YXZ'
                );

                if (data.frame) {
                    document.getElementById('vision-feed').src = 'data:image/jpeg;base64,' + data.frame;
                }
            };

            function sendCmd(cmd) { ws.send(JSON.stringify({ "command": cmd })); }

            function animate() {
                requestAnimationFrame(animate);
                controls.update();
                renderer.render(scene, camera);
            }
            animate();
        </script>
    </body>
    </html>
    """
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
            except asyncio.TimeoutError:
                pass

            t_in = 0.85 if flight_mode == "TAKEOFF" else (0.0 if flight_mode == "GLIDE" else 0.45)
            telemetry_data = uav.step(delta_thrust=t_in)
            
            # الحصول على البث البصري
            frame = vision.process_frame(telemetry_data)
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')

            payload = {
                "telemetry": telemetry_data,
                "mode": flight_mode,
                "frame": jpg_as_text
            }

            await websocket.send_text(json.dumps(payload))
            await asyncio.sleep(0.03)
    except WebSocketDisconnect:
        pass

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)