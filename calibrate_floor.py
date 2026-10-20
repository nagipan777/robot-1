import argparse
import re
from pathlib import Path

import cv2
import numpy as np


DEFAULT_FLOOR_POINTS = np.float32([
    [2.5, 1.0], [2.5, -1.0], [0.5, -1.0], [0.5, 1.0],
])
CORNER_NAMES = ["TOP-LEFT", "TOP-RIGHT", "BOTTOM-RIGHT", "BOTTOM-LEFT"]
VALIDATION_FLOOR_POINT = np.float32([1.5, 0.0])


def parse_point(value):
    try:
        numbers = [float(item) for item in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("px,py,x,y must be numeric") from exc
    if len(numbers) != 4:
        raise argparse.ArgumentTypeError("use px,py,x,y")
    return numbers


def compute_homography(pixels, floor):
    pixels = np.asarray(pixels, dtype=np.float32)
    if len(pixels) == 4 and not cv2.isContourConvex(pixels.reshape(-1, 1, 2)):
        raise ValueError("クリックした四隅が凸四角形になっていません。順序を確認してください")
    homography, _ = cv2.findHomography(
        pixels,
        np.asarray(floor, dtype=np.float32),
        method=0,
    )
    if homography is None or not np.isfinite(homography).all():
        raise ValueError("ホモグラフィを計算できません。四隅の配置を確認してください")
    homography /= homography[2, 2]
    if not np.isfinite(homography).all():
        raise ValueError("ホモグラフィに有限でない値が含まれています")
    if np.allclose(homography, np.eye(3), atol=1e-6):
        raise ValueError("単位行列は有効な校正として保存できません")
    return homography


def transform(points, homography):
    points = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(points, homography).reshape(-1, 2)


def verification(pixels, floor, homography):
    projected = transform(pixels, homography)
    errors = np.linalg.norm(projected - floor, axis=1)
    floor_center = np.mean(floor, axis=0)
    image_center = transform([floor_center], np.linalg.inv(homography))[0]
    checked_center = transform([image_center], homography)[0]
    return projected, errors, image_center, checked_center


def format_values(homography):
    values = []
    for value in homography.reshape(-1):
        text = f"{value:.10g}"
        if "." not in text and "e" not in text.lower():
            text += ".0"
        values.append(text)
    return ", ".join(values)


def save_to_controller_config(path, homography, image_size=None):
    config = path.read_text()
    replacement = f"image_to_odom_homography: [{format_values(homography)}]"
    config, count = re.subn(
        r"image_to_odom_homography:\s*\[[^\]]*\]", replacement, config
    )
    if count != 1:
        raise ValueError(f"{path} のホモグラフィ設定を一意に特定できません")
    config = re.sub(r"(control_enabled:\s*)\w+", r"\1false", config)
    config = re.sub(r"(calibration_ready:\s*)\w+", r"\1false", config)
    if image_size is not None:
        width, height = image_size
        config = re.sub(r"(calibration_image_width:\s*)\d+", rf"\g<1>{width}", config)
        config = re.sub(r"(calibration_image_height:\s*)\d+", rf"\g<1>{height}", config)
    path.write_text(config)


def print_verification(pixels, floor, homography):
    projected, errors, image_center, checked_center = verification(
        pixels, floor, homography
    )
    print(f"image_to_odom_homography: [{format_values(homography)}]")
    for index, (pixel, expected, actual, error) in enumerate(
        zip(pixels, floor, projected, errors), start=1
    ):
        print(
            f"{index}: pixel=({pixel[0]:.1f}, {pixel[1]:.1f}) "
            f"-> odom=({actual[0]:.3f}, {actual[1]:.3f}), "
            f"expected=({expected[0]:.3f}, {expected[1]:.3f}), error={error:.4f} m"
        )
    print(
        f"CENTER: pixel=({image_center[0]:.1f}, {image_center[1]:.1f}) "
        f"-> odom=({checked_center[0]:.3f}, {checked_center[1]:.3f})"
    )
    print(f"最大再投影誤差: {errors.max():.4f} m")
    print(f"平均再投影誤差: {errors.mean():.4f} m")
    return image_center


def draw_overlay(frame, pixels, floor, homography, validation_pixel=None, litter=None):
    height, width = frame.shape[:2]
    panel_width = 440
    view = np.zeros((height, width + panel_width, 3), dtype=frame.dtype)
    view[:, :width] = frame
    panel_x = width + 15
    cv2.putText(view, "Floor calibration", (panel_x, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv2.putText(view, "Click corners in order:", (panel_x, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    for index, (name, target) in enumerate(zip(CORNER_NAMES, floor)):
        color = (80, 220, 80) if index < len(pixels) else (220, 220, 220)
        text = f"{index + 1} {name} -> odom ({target[0]:.2f}, {target[1]:.2f}) m"
        cv2.putText(view, text, (panel_x, 86 + index * 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    for index, point in enumerate(pixels):
        pixel = tuple(np.rint(point).astype(int))
        cv2.circle(view, pixel, 7, (0, 255, 0), -1)
        cv2.putText(view, str(index + 1), (pixel[0] + 9, pixel[1] - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    if len(pixels) >= 2:
        cv2.polylines(view, [np.rint(pixels).astype(np.int32)], False, (0, 255, 0), 2)

    if litter is not None:
        litter_x, litter_y, confidence = litter
        litter_pixel = tuple(np.rint([litter_x, litter_y]).astype(int))
        cv2.drawMarker(view, litter_pixel, (255, 0, 255), cv2.MARKER_CROSS, 22, 2)
        cv2.putText(view, f"litter {confidence:.2f}",
                    (litter_pixel[0] + 10, litter_pixel[1] + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)
        cv2.putText(view, f"YOLO litter pixel ({litter_x:.1f}, {litter_y:.1f})",
                    (panel_x, 300), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 0, 255), 1)
        if homography is not None:
            litter_odom = transform([[litter_x, litter_y]], homography)[0]
            cv2.putText(view, f"odom ({litter_odom[0]:.3f}, {litter_odom[1]:.3f}) m",
                        (panel_x, 326), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 0, 255), 1)

    if homography is None:
        next_index = len(pixels)
        cv2.putText(view, f"NEXT: {next_index + 1} {CORNER_NAMES[next_index]}",
                    (panel_x, 230), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)
        cv2.putText(view, "Click inside the camera image", (panel_x, 262),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    else:
        if validation_pixel is not None:
            center_pixel = tuple(np.rint(validation_pixel).astype(int))
            actual = transform([validation_pixel], homography)[0]
            error = np.linalg.norm(actual - VALIDATION_FLOOR_POINT)
            cv2.drawMarker(view, center_pixel, (0, 165, 255), cv2.MARKER_CROSS, 24, 2)
            cv2.putText(view, f"VALIDATION error={error:.3f} m",
                        (center_pixel[0] + 12, center_pixel[1] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)
        cv2.putText(view, "CLICK validation point" if validation_pixel is None
                    else "CHECK: S=save  R=retry  Q=cancel",
                    (panel_x, 230), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 255), 2)
    return view


def detect_litter(model, frame, confidence):
    if model is None:
        return frame, None
    result = model.predict(frame, conf=confidence, verbose=False)[0]
    litter = []
    for box in result.boxes:
        if str(result.names[int(box.cls.item())]).lower() != "litter":
            continue
        x1, y1, x2, y2 = box.xyxy[0].cpu().tolist()
        litter.append(((x1 + x2) / 2, (y1 + y2) / 2, box.conf.item()))
    return result.plot(), max(litter, key=lambda item: item[2], default=None)


def interactive_calibration(camera, floor, output, model=None, confidence=0.5):
    capture = cv2.VideoCapture(camera)
    if not capture.isOpened():
        raise SystemExit(f"カメラ {camera} を開けません")

    window = "Floor calibration"
    pixels = []
    homography = None
    validation_pixel = None
    image_size = [0, 0]
    display_scale = [1.0]

    def on_mouse(event, x, y, _flags, _data):
        nonlocal homography, validation_pixel
        image_x = x / display_scale[0]
        image_y = y / display_scale[0]
        if (
            event != cv2.EVENT_LBUTTONUP
            or image_x < 0
            or image_y < 0
            or image_x >= image_size[0]
            or image_y >= image_size[1]
        ):
            return
        if homography is not None:
            if validation_pixel is not None:
                return
            validation_pixel = np.float32([image_x, image_y])
            actual = transform([validation_pixel], homography)[0]
            error = float(np.linalg.norm(actual - VALIDATION_FLOOR_POINT))
            print(
                f"独立検証点: pixel=({image_x:.1f}, {image_y:.1f}) -> "
                f"odom=({actual[0]:.3f}, {actual[1]:.3f}), "
                f"expected=(1.500, 0.000), error={error:.4f} m"
            )
            print("検証点誤差を確認し、Sで保存、Rでやり直してください")
            return
        if len(pixels) >= 4:
            return
        pixels.append([float(image_x), float(image_y)])
        target = floor[len(pixels) - 1]
        print(
            f"{len(pixels)} {CORNER_NAMES[len(pixels) - 1]}: "
            f"pixel=({image_x:.1f}, {image_y:.1f}) -> "
            f"odom=({target[0]:.2f}, {target[1]:.2f})"
        )
        if len(pixels) == 4:
            try:
                homography = compute_homography(pixels, floor)
                print_verification(np.float32(pixels), floor, homography)
                print("実測した中央の検証点 (1.50, 0.00) をクリックしてください")
            except ValueError as exc:
                print(f"校正エラー: {exc}")
                pixels.clear()
                homography = None

    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window, on_mouse)
    print("作業領域: x=0.5..2.5 m, y=-1.0..1.0 m（ロボットとドックを除外）")
    print("画像上の床領域を TOP-LEFT, TOP-RIGHT, BOTTOM-RIGHT, BOTTOM-LEFT の順でクリックしてください")
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                raise SystemExit("カメラ画像を取得できません")
            image_size[:] = [frame.shape[1], frame.shape[0]]
            annotated, litter = detect_litter(model, frame, confidence)
            view = draw_overlay(
                annotated, np.float32(pixels), floor, homography,
                validation_pixel, litter,
            )
            display_scale[0] = min(1.0, 1400 / view.shape[1], 850 / view.shape[0])
            if display_scale[0] < 1.0:
                view = cv2.resize(view, None, fx=display_scale[0], fy=display_scale[0])
            cv2.imshow(window, view)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                print("校正をキャンセルしました")
                return False
            if key == ord("r"):
                pixels.clear()
                homography = None
                validation_pixel = None
                print("クリック点をリセットしました")
            if key == ord("s") and homography is not None and validation_pixel is not None:
                save_to_controller_config(output, homography, image_size)
                print(f"校正結果を保存しました: {output}")
                print("control_enabled=false, calibration_ready=false を維持しました")
                return True
            if key == ord("s") and homography is not None and validation_pixel is None:
                print("保存前に独立検証点 (1.50, 0.00) をクリックしてください")
    finally:
        capture.release()
        cv2.destroyAllWindows()


def main():
    default_output = Path(__file__).resolve().parent / "simulation/vision_controller.yaml"
    parser = argparse.ArgumentParser(
        description="固定カメラの画素座標からGazebo床面座標へのホモグラフィを計算・保存"
    )
    parser.add_argument("--camera", type=int, default=0, help="固定Webカメラ番号")
    parser.add_argument("--output", type=Path, default=default_output, help="保存先の制御設定")
    parser.add_argument(
        "--weights", type=Path,
        default=Path(__file__).resolve().parent / "runs/detect/taco/weights/best.pt",
        help="校正画面へ重ねるYOLOモデル",
    )
    parser.add_argument("--confidence", type=float, default=0.5, help="YOLO信頼度しきい値")
    parser.add_argument("--no-yolo", action="store_true", help="YOLO表示を無効にする")
    parser.add_argument(
        "--point", type=parse_point, action="append", metavar="PX,PY,X,Y",
        help="GUIを使わない場合の対応点。4組以上指定",
    )
    args = parser.parse_args()

    if args.point:
        if len(args.point) < 4:
            parser.error("--point は4組以上必要です")
        pixels = np.float32([[point[0], point[1]] for point in args.point])
        floor = np.float32([[point[2], point[3]] for point in args.point])
        try:
            homography = compute_homography(pixels, floor)
        except ValueError as exc:
            parser.error(str(exc))
        print_verification(pixels, floor, homography)
        save_to_controller_config(args.output, homography)
        print(f"校正結果を保存しました: {args.output}")
        print("control_enabled=false, calibration_ready=false を維持しました")
        return

    model = None
    if not args.no_yolo:
        if not args.weights.is_file():
            parser.error(f"YOLOモデルが見つかりません: {args.weights}")
        from ultralytics import YOLO

        model = YOLO(args.weights)
        if model.task != "detect":
            parser.error("物体検出用のYOLOモデルを指定してください")
        print(f"YOLOモデルを読み込みました: {args.weights}")
    interactive_calibration(
        args.camera, DEFAULT_FLOOR_POINTS, args.output, model, args.confidence
    )


if __name__ == "__main__":
    main()
