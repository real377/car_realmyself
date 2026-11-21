"""
step3_verify.py
验证 kmodel 和 ONNX 输出余弦相似度，>0.99 即合格
"""

import os
import cv2
import numpy as np
import onnxruntime as ort
import nncase

# -------------------- 可调参数 --------------------
ONNX_PATH = r"D:\deeplearning\ultralytics-8.3.163\runs\detect\train7\weights\best.onnx"
KMODEL_PATH = r"D:\deeplearning\ultralytics-8.3.163\runs\detect\train7\weights\best.kmodel"
TEST_IMG_DIR = r"D:\deeplearning\make_dataset\xz_dataset\images\val"  # 验证集里拿一张测试
INPUT_WIDTH = 640
INPUT_HEIGHT = 192
# ---------------------------------------------------

def get_onnx_input(img_path, model_input_size):
    """ONNX 输入：BGR→RGB + resize + /255 + NCHW"""
    img = cv2.imread(img_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (model_input_size[0], model_input_size[1]))
    img = np.asarray(img, dtype=np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    return img.copy()

def get_kmodel_input(img_path, model_input_size):
    """kmodel 输入：BGR + resize + uint8 + NCHW（preprocess 在模型内部）"""
    img = cv2.imread(img_path)  # BGR
    img = cv2.resize(img, (model_input_size[0], model_input_size[1]))
    img = np.asarray(img, dtype=np.uint8)
    img = np.transpose(img, (2, 0, 1))
    return img.copy()

def onnx_inference(onnx_path, input_data):
    ort_session = ort.InferenceSession(onnx_path)
    output_names = [o.name for o in ort_session.get_outputs()]
    model_input = ort_session.get_inputs()[0]
    input_data = input_data.astype(np.float32).reshape(model_input.shape)
    return ort_session.run(output_names, {model_input.name: input_data})

def kmodel_inference(kmodel_path, input_data, model_input_size):
    sim = nncase.Simulator()
    with open(kmodel_path, 'rb') as f:
        sim.load_model(f.read())
    input_shape = [1, 3, model_input_size[1], model_input_size[0]]
    dtype = sim.get_input_desc(0).dtype
    kmodel_input = input_data.astype(dtype).reshape(input_shape)
    sim.set_input_tensor(0, nncase.RuntimeTensor.from_numpy(kmodel_input))
    sim.run()
    results = [sim.get_output_tensor(i).to_numpy() for i in range(sim.outputs_size)]
    return results

def cosine_similarity(onnx_results, kmodel_results):
    for i in range(len(kmodel_results)):
        a = np.reshape(onnx_results[i], (-1))
        b = np.reshape(kmodel_results[i], (-1))
        cos = (a @ b) / (np.linalg.norm(a, 2) * np.linalg.norm(b, 2))
        status = "✅ 通过" if cos > 0.99 else "❌ 不通过"
        print(f'output {i} 余弦相似度: {cos:.6f}  {status}')

def main():
    imgs = [f for f in os.listdir(TEST_IMG_DIR) if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))]
    if not imgs:
        raise FileNotFoundError(f"{TEST_IMG_DIR} 中没有图片！")
    test_img = os.path.join(TEST_IMG_DIR, imgs[0])
    print(f"测试图片: {test_img}")

    model_input_size = [INPUT_WIDTH, INPUT_HEIGHT]

    print("ONNX 推理...")
    onnx_out = onnx_inference(ONNX_PATH, get_onnx_input(test_img, model_input_size))

    print("kmodel 推理...")
    kmodel_out = kmodel_inference(KMODEL_PATH, get_kmodel_input(test_img, model_input_size), model_input_size)

    cosine_similarity(onnx_out, kmodel_out)

if __name__ == "__main__":
    main()