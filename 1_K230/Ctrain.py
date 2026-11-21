# train_ball_yolov5nu.py
from ultralytics import YOLO

if __name__ == '__main__':
    # YOLOv5nu 用 yolo11n.pt 做预训练？不，YOLOv5 有自己的预训练。
    # 官方提供 yolov5nu.pt，下载后放到项目根目录。
    model = YOLO("yolov5nu.pt")          # 需要先下载这个权重

    model.train(
        data="ball.yaml",
        epochs=150,
        imgsz=224,                       # 保持和之前一样 
        batch=16,
        workers=2,                       

        lr0=0.01,
        lrf=0.01,
        cos_lr=True,

        momentum=0.937,
        weight_decay=0.0005,
        warmup_epochs=3,

        box=7.5,
        cls=0.5,
        dfl=1.5,                         # YOLOv5 默认没有 dfl，但 ultralytics 的 YOLOv5 实现可能有，保持即可

        # 数据增强
        mosaic=1.0,
        mixup=0.0,
        degrees=5.0,
        scale=0.5,
        fliplr=0.5,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,

        # YOLOv5 特有的参数
        rect=False,                      # 禁止矩形训练

        single_cls=True,                # 单类别就设 True，可以提升收敛速度
    )
