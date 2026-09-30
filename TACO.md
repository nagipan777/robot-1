# TACOでゴミ検出を学習する

`prepare_taco.py` は https://huggingface.co/datasets/pictograph/taco の画像と注釈を取得し、YOLO用の `taco_dataset/data.yaml` を生成します。

```bash
cd /Users/imaichika/robot
source .venv/bin/activate
python prepare_taco.py
yolo detect train model=yolo26n.pt data=taco_dataset/data.yaml epochs=40 imgsz=640 device=mps name=taco
```

学習完了後、ログに表示された保存先の `weights/best.pt` を指定します。初回の保存先の場合：

```bash
python vision.py --camera 0 --weights runs/detect/taco/weights/best.pt
```

同じ名前で繰り返し学習すると保存先に番号が付く場合があります。

全カテゴリを `0: litter`（ゴミ）に統合しています。床のシミ・ホコリを学習したモデルではありません。既存の `dirt_dataset` は変更しません。画像単位で固定シード42を使い、学習80%・検証20%に分割します。類似した撮影場所が両方に含まれる可能性があるため、実際のカメラ環境で別途評価してください。

ダウンロード失敗時は同じコマンドで再実行できます。取得済みの画像は再利用します。途中のデータで学習しないよう、全件の変換が完了してから `data.yaml` を生成します。

出典・ライセンス情報は生成先の `SOURCE.txt` に記録します。
