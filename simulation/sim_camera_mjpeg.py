#!/usr/bin/env python3
"""Expose the simulated fixed camera as a localhost-published MJPEG stream."""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class LatestFrame:
    """Thread-safe storage for the most recent encoded JPEG frame."""

    def __init__(self):
        self._condition = threading.Condition()
        self._jpeg = None
        self._sequence = 0

    def update(self, jpeg):
        with self._condition:
            self._jpeg = jpeg
            self._sequence += 1
            self._condition.notify_all()

    def wait_after(self, sequence, timeout=2.0):
        with self._condition:
            self._condition.wait_for(
                lambda: self._sequence > sequence,
                timeout=timeout,
            )
            return self._sequence, self._jpeg

    @property
    def ready(self):
        with self._condition:
            return self._jpeg is not None

    def latest(self):
        with self._condition:
            return self._sequence, self._jpeg


class FixedCameraMjpegNode(Node):
    def __init__(self, frames):
        super().__init__("fixed_camera_mjpeg")
        self._frames = frames
        self._bridge = CvBridge()
        self.create_subscription(
            Image,
            "/sim/fixed_camera/image_raw",
            self._image_callback,
            qos_profile_sensor_data,
        )

    def _image_callback(self, message):
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, 85],
            )
            if not ok:
                raise RuntimeError("OpenCV could not encode the camera image")
            self._frames.update(encoded.tobytes())
        except Exception as exc:  # Keep the previous valid frame on conversion failure.
            self.get_logger().error(f"camera frame conversion failed: {exc}")


def make_handler(frames):
    class MjpegHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/health":
                sequence, _jpeg = frames.latest()
                payload = json.dumps(
                    {"ready": frames.ready, "sequence": sequence}
                ).encode("utf-8")
                self.send_response(200 if frames.ready else 503)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return

            if self.path == "/snapshot.jpg":
                _sequence, jpeg = frames.latest()
                if jpeg is None:
                    self.send_error(503, "camera frame is not ready")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(jpeg)
                return

            if self.path != "/stream.mjpg":
                self.send_error(404)
                return

            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            sequence = 0
            try:
                while True:
                    next_sequence, jpeg = frames.wait_after(sequence)
                    if jpeg is None or next_sequence == sequence:
                        continue
                    sequence = next_sequence
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                    )
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, _format, *_args):
            return

    return MjpegHandler


def main():
    frames = LatestFrame()
    server = ThreadingHTTPServer(("0.0.0.0", 8766), make_handler(frames))
    server.daemon_threads = True
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    rclpy.init()
    node = FixedCameraMjpegNode(frames)
    node.get_logger().info(
        "MJPEG server listening on port 8766 for /sim/fixed_camera/image_raw"
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
