#!/usr/bin/env python3
import json
import threading
import sys

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

import websocket


if ROS2_AVAILABLE:
    class StealthBWBROS2Bridge(Node):
        def __init__(self):
            super().__init__('stealth_bwb_ros2_bridge')
            self.pose_pub = self.create_publisher(PoseStamped, '/uav/stealth/pose', 10)
            self.path_pub = self.create_publisher(Path, '/uav/stealth/path', 10)
            self.start_ws()

        def start_ws(self):
            def run():
                ws = websocket.WebSocketApp("ws://localhost:8000/ws/telemetry", on_message=self.on_message)
                ws.run_forever()
            threading.Thread(target=run, daemon=True).start()

        def on_message(self, ws, message):
            data = json.loads(message).get("telemetry", {})
            stamp = self.get_clock().now().to_msg()
            
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "map"
            pos = data.get("position", [0, 0, 0])
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(pos[0]), float(pos[1]), float(pos[2])
            self.pose_pub.publish(pose)


def main():
    if ROS2_AVAILABLE:
        rclpy.init()
        node = StealthBWBROS2Bridge()
        rclpy.spin(node)
    else:
        print("[INFO] ROS 2 (rclpy) غير متوفر. يعمل النظام في الوضع المستقل.")

if __name__ == '__main__':
    main()