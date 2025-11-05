import torch

engine_path = "./yolox_cheng.engine"
output_path = "./model_trt.pth"

# 读取 engine 的二进制数据
with open(engine_path, "rb") as f:
    engine_bytes = f.read()

# 构造与 torch2trt 一致的结构
state_dict = {"engine": engine_bytes}

# 保存为 .pth（或改成 .pkl 也行）
torch.save(state_dict, output_path)
print(f"✅ 已成功将 {engine_path} 封装为 {output_path}")
