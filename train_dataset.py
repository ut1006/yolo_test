from ultralytics import YOLO

def main():
    # YOLOv8のセグメンテーションモデル（最小モデルサイズ）をロード
    model = YOLO("yolov8s-seg.pt")  # 他にも 'yolov8s-seg.pt', 'yolov8m-seg.pt' などが選べます

    # 学習を実行
    results = model.train(
        data="dataset.yaml",        # データセット設定ファイルのパス
        epochs=50,                  # エポック数（例として50に設定）
        imgsz=1152,                 # 画像サイズ
        batch=-1,                   # バッチサイズ
        name="lab9F_segmentation",  # モデル保存時の名前
        device="cuda"               # 使用するデバイス（GPU）
    )
    
    # 学習結果の確認（オプション）
    print("Training results:", results)

if __name__ == '__main__':
    main()
