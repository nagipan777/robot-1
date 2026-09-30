#!/usr/bin/env python3
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WebcamBridge(Node):
    def __init__(self):
        super().__init__("webcam_bridge")
        self.publisher = self.create_publisher(String, "/webcam/detections", 10)

    def publish(self, payload):
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.publisher.publish(message)


class RequestHandler(BaseHTTPRequestHandler):
    bridge = None

    def do_GET(self):
        if self.path != "/health":
            self.send_error(404)
            return
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b"ok\n")

    def do_POST(self):
        if self.path != "/detections":
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 1_000_000:
                raise ValueError("invalid Content-Length")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict) or not isinstance(payload.get("detections"), list):
                raise ValueError("detections must be a list")
            self.bridge.publish(payload)
        except (ValueError, json.JSONDecodeError) as exc:
            self.send_error(400, str(exc))
            return
        self.send_response(204)
        self.end_headers()

    def log_message(self, _format, *_args):
        return


def main():
    rclpy.init()
    bridge = WebcamBridge()
    RequestHandler.bridge = bridge
    server = ThreadingHTTPServer(("0.0.0.0", 8765), RequestHandler)
    bridge.get_logger().info("Webカメラ接続をポート8765で待機しています")
    try:
        server.serve_forever()
    finally:
        server.server_close()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
