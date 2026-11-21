"""
step1_export_onnx.py
将 YOLO11n 训练的 best.pt 导出为 ONNX 格式
"""

from ultralytics import YOLO

# -------------------- 可调参数 --------------------
PT_PATH = r"D:\deeplearning\ultralytics-8.3.163\runs\detect\train3\weights\best.pt"
IMGSZ = 224           # 必须和训练时一致
OPSET = 11            # nncase 2.11 推荐 opset=11
# ---------------------------------------------------

def main():
    print("加载 .pt 模型...")
    model = YOLO(PT_PATH)

    print(f"导出 ONNX（imgsz={IMGSZ}, opset={OPSET}）...")
    model.export(
        format="onnx",
        imgsz=IMGSZ,
        opset=OPSET,
        simplify=True,      # 自动简化模型结构
        dynamic=False,      # 固定输入尺寸
        half=False,         # 不启用 FP16
        batch=1,            # 单张推理
        nms=False,          # NMS 在开发板端做
    )
    print("ONNX 导出完成！会在 best.pt 同目录生成 best.onnx")

if __name__ == "__main__":
    main()