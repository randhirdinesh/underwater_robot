import torch
import tensorrt as trt
import torch.onnx
from torch2trt import torch2trt

# Load trained PyTorch model
model = torch.load("models/sonar_ai_model.pth")
model.eval()

# Convert PyTorch model to ONNX
dummy_input = torch.randn(1, 64)  # Example input shape
torch.onnx.export(model, dummy_input, "models/sonar_ai_model.onnx")

# Convert ONNX to TensorRT
logger = trt.Logger(trt.Logger.WARNING)
builder = trt.Builder(logger)
network = builder.create_network()
parser = trt.OnnxParser(network, logger)
with open("models/sonar_ai_model.onnx", "rb") as model_file:
    parser.parse(model_file.read())

engine = builder.build_cuda_engine(network)

# Save the TensorRT engine
with open("models/sonar_ai_model.trt", "wb") as engine_file:
    engine_file.write(engine.serialize())

print("Sonar AI model converted to TensorRT successfully!")
