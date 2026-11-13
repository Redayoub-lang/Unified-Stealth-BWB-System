#!/usr/bin/env python3
import json
import math
import threading
import time
from collections import deque

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

try:
    import websocket
    WS_AVAILABLE = True
except ImportError:
    WS_AVAILABLE = False


def euler_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple[float, float, float, float]:
    """
    تحويل زوايا أويلر بالدرجات (Roll, Pitch, Yaw) إلى رباعي دوران (Quaternion: x, y, z, w)
    متوافق مع معيار ROS 2 بدون الحاجة لمكتبات خارجية.
    """
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


if ROS2_AVAILABLE:
    class StealthBWBROS2Bridge(Node):
        """
        جسر ربط بيانات القياس (Telemetry) مع شبكة ROS 2 لتنسيق الأسراب والملاحة الذاتية.
        """

        def __init__(self):
            super().__init__('stealth_bwb_ros2_bridge')

            # معاملات قابلة للتعديل عند التشغيل الميداني أو تعدد الطائرات (Swarm)
            self.declare_parameter('ws_url', 'ws://localhost:8000/ws/telemetry')
            self.declare_parameter('uav_id', 'stealth')
            self.declare_parameter('max_path_points', 500)

            self.ws_url = self.get_parameter('ws_url').get_parameter_value().string_value
            self.uav_id = self.get_parameter('uav_id').get_parameter_value().string_value
            max_points = self.get_parameter('max_path_points').get_parameter_value().integer_value

            # قنوات النشر (Topics) الخاصة بالطائرة في ROS 2
            self.pose_pub = self.create_publisher(PoseStamped, f'/uav/{self.uav_id}/pose', 10)
            self.path_pub = self.create_publisher(Path, f'/uav/{self.uav_id}/path', 10)

            # تخزين آخر 500 نقطة من مسار الطيران لمنع امتلاء الذاكرة
            self.path_history = deque(maxlen=max_points)
            self.running = True
            self.ws_app = None

            self.get_logger().info(f"ROS 2 Bridge initialized for '{self.uav_id}' -> {self.ws_url}")
            self.start_ws()

        def start_ws(self):
            """
            تشغيل اتصال WebSocket في خيط مستقل مع إعادة الاتصال التلقائي عند الانقطاع.
            """
            def run():
                while self.running and rclpy.ok():
                    try:
                        self.ws_app = websocket.WebSocketApp(
                            self.ws_url,
                            on_open=lambda ws: self.get_logger().info("Connected to C2 Telemetry Stream."),
                            on_message=self.on_message,
                            on_error=lambda ws, err: self.get_logger().warn(f"WebSocket error: {err}"),
                            on_close=lambda ws, code, msg: self.get_logger().warn("Telemetry stream closed. Reconnecting in 2s...")
                        )
                        self.ws_app.run_forever(ping_interval=10, ping_timeout=5)
                    except Exception as exc:
                        self.get_logger().error(f"Connection failure: {exc}")

                    if self.running:
                        time.sleep(2.0)

            threading.Thread(target=run, daemon=True).start()

        def on_message(self, ws, message: str):
            """
            استقبال حزمة Telemetry وتحويلها إلى رسائل PoseStamped و Path في ROS 2.
            """
            try:
                payload = json.loads(message)
                data = payload.get("telemetry", payload)
                stamp = self.get_clock().now().to_msg()

                # 1. استخراج الموقع بأمان (دعم pos و position)
                pos = data.get("pos", data.get("position", [0.0, 0.0, 0.0]))
                euler_deg = data.get("euler_deg", [0.0, 0.0, 0.0])

                # 2. بناء رسالة الموقع والاتجاه PoseStamped
                pose = PoseStamped()
                pose.header.stamp = stamp
                pose.header.frame_id = "map"

                pose.pose.position.x = float(pos[0])
                pose.pose.position.y = float(pos[1])
                pose.pose.position.z = float(pos[2])

                qx, qy, qz, qw = euler_to_quaternion(
                    float(euler_deg[0]),
                    float(euler_deg[1]),
                    float(euler_deg[2])
                )
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

                self.pose_pub.publish(pose)

                # 3. تحديث ونشر مسار الرحلة المتراكم Path
                self.path_history.append(pose)
                path_msg = Path()
                path_msg.header.stamp = stamp
                path_msg.header.frame_id = "map"
                path_msg.poses = list(self.path_history)
                self.path_pub.publish(path_msg)

            except (json.JSONDecodeError, IndexError, KeyError, ValueError) as exc:
                self.get_logger().debug(f"Malformed telemetry packet skipped: {exc}")

        def shutdown(self):
            """
            إغلاق الاتصال والخيوط البرمجية بأمان.
            """
            self.running = False
            if self.ws_app is not None:
                self.ws_app.close()


def main():
    if not ROS2_AVAILABLE:
        print("[INFO] ROS 2 (rclpy) غير متوفر على نظام Windows. يعمل النظام في الوضع المستقل (Standalone).")
        return

    if not WS_AVAILABLE:
        print("[ERROR] مكتبة websocket-client غير مثبتة. نفّذ: pip install websocket-client")
        return

    rclpy.init()
    node = StealthBWBROS2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()