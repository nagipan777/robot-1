import argparse
import json
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2


def select_video_source(camera, video_url):
    """Return the OpenCV source while preserving camera 0 as the default."""
    return video_url if video_url else camera


class SimulatorConnection:
    def __init__(self, url):
        self.url = url
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
            with urllib.request.urlopen(request, timeout=0.2):
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
        "--weights",
        default=str(Path(__file__).resolve().parent / "runs/detect/taco/weights/best.pt"),
        help="学習済みモデルのパス（既定: TACOのbest.pt）",
    )
    args = parser.parse_args()
    simulator = SimulatorConnection(args.simulator) if args.simulator else None

    model = None
    if not args.preview:
        if not Path(args.weights).is_file():
            parser.error("学習済みモデルが見つかりません。--weights で指定するか、--preview で映像を確認してください。")
        from ultralytics import YOLO

        model = YOLO(args.weights)
        if model.task != "detect":
            parser.error("物体検出用のYOLOモデルを --weights で指定してください。")
        print(f"YOLOモデルを読み込みました: {args.weights}")

    video_source = select_video_source(args.camera, args.video_url)
    source_name = args.video_url if args.video_url else f"カメラ {args.camera}"
    cap = cv2.VideoCapture(video_source)
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
                print("映像を取得しました。映像ウィンドウで q または Esc を押すと終了します。")
                received_frame = True

            detections = []
            if model is not None:
                result = model.predict(frame, conf=0.5, verbose=False)[0]
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

            if simulator is not None:
                simulator.send(
                    {
                        "timestamp": time.time(),
                        "image_width": int(frame.shape[1]),
                        "image_height": int(frame.shape[0]),
                        "detections": detections,
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
