from ultralytics import YOLO

model = YOLO(r"D:\deeplearning\ultralytics-8.3.163\runs\detect\train2\weights\best.pt")
model.predict(
    source=r"D:\deeplearning\make_dataset\images",
    save=True,
    show=False,
    save_txt=True,
    conf=0.5,
    iou=0.4,
    max_det=10,
)