from yolox.exp import get_exp
import torch

# Load your custom YOLOX model
exp = get_exp("yolox_x_mushroom.py")  # Replace with your exp file
model = exp.get_model()
model.load_state_dict(torch.load("yolox_x_mushroom.pth", map_location="cpu"))
model.eval()

# Dummy input (adjust shape to your model)
dummy_input = torch.randn(1, 3, 640, 640).to("cuda" if torch.cuda.is_available() else "cpu")

# Export to ONNX
torch.onnx.export(
    model,
    dummy_input,
    "yolox_x_mushroom.onnx",
    input_names=["images"],
    output_names=["output"],
    dynamic_axes={"images": {0: "batch"}, "output": {0: "batch"}},
    opset_version=11
)
