from ultralytics import YOLO, RTDETR

# Load a model
model = RTDETR('./rtdetr.pt')  # load an official model

# Export the model
model.export(format='engine', imgsz=(736,1280), int8=True, simplify=True)