from yolox.exp import get_exp
import torch

# Load your custom YOLOX model
exp = get_exp("/home/jimmy/Downloads/mushroomproject/YOLOX-main/exps/example/yolox_voc/yolox_voc_s.py")  # Replace with your exp file
model = exp.get_model()
model.load_state_dict(torch.load("/home/jimmy/Downloads/yolox_chengcheng.pth", map_location="cpu"),strict=False)
model.eval()

# Dummy input (adjust shape to your model)
dummy_input = torch.randn(1, 3, 640, 640).to("cuda" if torch.cuda.is_available() else "cpu")
device = "cuda"
model.to(device)
# Export to ONNX
torch.onnx.export(
    model,
    dummy_input,
    "/home/jimmy/Downloads/yolox_chengcheng.onnx",
    input_names=["images"],
    output_names=["output"],
    dynamic_axes={"images": {0: "batch"}, "output": {0: "batch"}},
    opset_version=11
)
