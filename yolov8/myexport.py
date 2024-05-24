from ultralytics import YOLO

 

model = YOLO('best_v3.pt')  # load a custom trained

 

# Export the model

model.export(format='engine',imgsz=(720,1280),half=True,simplify=True,device="0")
