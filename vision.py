import argparse
import json
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2


DEFAULT_CONFIDENCE = 0.5


def confidence_threshold(value):
    """Parse a YOLO confidence threshold accepted by the CLI."""
    threshold = float(value)
    if not 0.0 <= threshold <= 1.0:
        raise argparse.ArgumentTypeError("confidence must be between 0 and 1")
    return threshold


def select_video_source(camera, video_url):
    """Return the OpenCV source while preserving camera 0 as the default."""
    return video_url if video_url else camera


def open_video_capture(source, network_stream=False):
    """Open a camera, bounding network-stream stalls so it can reconnect."""
    if network_stream:
        return cv2.VideoCapture(
            source,
            cv2.CAP_FFMPEG,
            [
                cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
                5000,
                cv2.CAP_PROP_READ_TIMEOUT_MSEC,
                5000,
            ],
        )
    return cv2.VideoCapture(source)


def create_aruco_detector(dictionary_name):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV ArUco support is unavailable; install opencv-contrib-python")
    dictionary_id = getattr(cv2.aruco, dictionary_name, None)
    if dictionary_id is None or not dictionary_name.startswith("DICT_"):
        raise ValueError("unsupported ArUco dictionary: {}".format(dictionary_name))
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    parameters = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def detect_aruco(detector, frame, marker_id):
    corners, ids, _rejected = detector.detectMarkers(frame)
    detections = []
    if ids is None:
        return detections, corners, ids
    for marker_corners, detected_id in zip(corners, ids.flatten()):
        if int(detected_id) != marker_id:
            continue
        detections.append(
            {
                "marker_id": int(detected_id),
                "corners": [
                    [round(float(point[0]), 2), round(float(point[1]), 2)]
                    for point in marker_corners.reshape(4, 2)
                ],
            }
        )
    return detections, corners, ids


class SimulatorConnection:
    def __init__(self, url, timeout=0.2):
        self.url = url
        self.timeout = timeout
        self.last_warning_at = 0.0
        self.connected = False

    def send(self, payload):
        request = urllib.request.Request(
            self.url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout):
                pass
            if not self.connected:
                print(f"シミュレーターに接続しました: {self.url}")
                self.connected = True
        except (OSError, urllib.error.URLError) as exc:
            now = time.monotonic()
            if self.connected or now - self.last_warning_at >= 10:
                print(f"シミュレーターに送信できません: {exc}")
                print("先に ./simulation/sim.sh up を実行してください。")
                self.last_warning_at = now
            self.connected = False


def main():
    parser = argparse.ArgumentParser(description="Webカメラの映像表示・YOLOによるゴミ検出")
    source_group = parser.add_mutually_exclusive_group()
    source_group.add_argument("--camera", type=int, default=0, help="カメラ番号（既定: 0）")
    source_group.add_argument(
        "--video-url",
        help="MJPEGなどの映像URL（例: http://127.0.0.1:8766/stream.mjpg）",
    )
    parser.add_argument("--width", type=int, default=1280, help="要求する画像幅（既定: 1280）")
    parser.add_argument("--height", type=int, default=960, help="要求する画像高さ（既定: 960）")
    parser.add_argument("--preview", action="store_true", help="検出せずにカメラ映像のみ表示")
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="録画時などにフレームごとの検出ログを表示しない",
    )
    parser.add_argument(
        "--simulator",
        default="http://127.0.0.1:8765/detections",
        help="検出結果の送信先（空文字を指定すると送信しない）",
    )
    parser.add_argument(
        "--http-timeout",
        type=float,
        default=0.2,
        help="検出JSON送信のタイムアウト秒（既定: 0.2）",
    )
    parser.add_argument(
        "--weights",
        default=str(Path(__file__).resolve().parent / "runs/detect/taco/weights/best.pt"),
        help="学習済みモデルのパス（既定: TACOのbest.pt）",
    )
    parser.add_argument(
        "--confidence",
        type=confidence_threshold,
        default=DEFAULT_CONFIDENCE,
        help="YOLO検出信頼度の下限（既定: 0.5）",
    )
    parser.add_argument(
        "--aruco-marker-id",
        type=int,
        default=-1,
        help="ArUco ID (-1 disables detection; default: -1)",
    )
    parser.add_argument(
        "--aruco-dictionary",
        default="DICT_4X4_50",
        help="OpenCV ArUco dictionary name (default: DICT_4X4_50)",
    )
    args = parser.parse_args()
    if args.http_timeout <= 0.0:
        parser.error("--http-timeout must be greater than zero")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width and --height must be positive")
    simulator = (
        SimulatorConnection(args.simulator, args.http_timeout)
        if args.simulator
        else None
    )

    model = None
    if not args.preview:
        if not Path(args.weights).is_file():
            parser.error("学習済みモデルが見つかりません。--weights で指定するか、--preview で映像を確認してください。")
        from ultralytics import YOLO

        model = YOLO(args.weights)
        if model.task != "detect":
            parser.error("物体検出用のYOLOモデルを --weights で指定してください。")
        print(f"YOLOモデルを読み込みました: {args.weights}")
    aruco_detector = None
    if args.aruco_marker_id >= 0:
        try:
            aruco_detector = create_aruco_detector(args.aruco_dictionary)
        except (RuntimeError, ValueError) as exc:
            parser.error(str(exc))
        print(
            "ArUco detector enabled: dictionary={} marker_id={}".format(
                args.aruco_dictionary, args.aruco_marker_id
            )
        )

    video_source = select_video_source(args.camera, args.video_url)
    source_name = args.video_url if args.video_url else f"カメラ {args.camera}"
    cap = open_video_capture(video_source, network_stream=bool(args.video_url))
    if not args.video_url:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    try:
        if not cap.isOpened():
            raise SystemExit(
                f"映像入力 {source_name} に接続できません。"
                "入力URL、USB接続、カメラへのアクセス許可を確認してください。"
            )
        print(f"{source_name} の映像を待っています…")
        last_frame_at = time.monotonic()
        received_frame = False
        while True:
            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                if args.video_url:
                    if time.monotonic() - last_frame_at >= 10:
                        print(
                            f"映像入力 {source_name} が途切れました。再接続します。"
                        )
                        last_frame_at = time.monotonic()
                    cap.release()
                    time.sleep(0.5)
                    cap = open_video_capture(video_source, network_stream=True)
                    continue
                if time.monotonic() - last_frame_at >= 10:
                    raise SystemExit(
                        f"映像入力 {source_name} から10秒間映像を取得できませんでした。"
                        "URLまたはUSB接続を確認してください。実カメラの場合は、"
                        "他のカメラ使用アプリを終了し、--camera 1 なども試してください。"
                    )
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
                time.sleep(0.1)
                continue
            last_frame_at = time.monotonic()
            if not received_frame:
                actual_height, actual_width = frame.shape[:2]
                if (actual_width, actual_height) != (args.width, args.height):
                    raise SystemExit(
                        f"カメラ解像度が校正条件と異なります: "
                        f"要求 {args.width}x{args.height}, 実際 {actual_width}x{actual_height}"
                    )
                print("映像を取得しました。映像ウィンドウで q または Esc を押すと終了します。")
                received_frame = True

            detections = []
            aruco_detections = []
            aruco_corners = None
            aruco_ids = None
            if aruco_detector is not None:
                aruco_detections, aruco_corners, aruco_ids = detect_aruco(
                    aruco_detector, frame, args.aruco_marker_id
                )
            if model is not None:
                result = model.predict(frame, conf=args.confidence, verbose=False)[0]
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().tolist()
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    label = result.names[int(box.cls.item())]
                    confidence = box.conf.item()
                    detections.append(
                        {
                            "label": label,
                            "confidence": round(confidence, 4),
                            "center_x": round(center_x, 1),
                            "center_y": round(center_y, 1),
                            "width": round(x2 - x1, 1),
                            "height": round(y2 - y1, 1),
                        }
                    )
                    if not args.quiet:
                        print(
                            f"{label} 信頼度: {confidence:.2f} "
                            f"画像座標: ({center_x:.0f}, {center_y:.0f})"
                        )
                frame = result.plot()
            if aruco_detector is not None and aruco_ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, aruco_corners, aruco_ids)

            if simulator is not None:
                simulator.send(
                    {
                        "timestamp": time.time(),
                        "image_width": int(frame.shape[1]),
                        "image_height": int(frame.shape[0]),
                        "detections": detections,
                        "aruco_detections": aruco_detections,
                    }
                )

            cv2.imshow("YOLO litter detection", frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
