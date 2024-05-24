from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor
import cv2
print('hi')

model =YOLO('best.pt')#YOLO('yolov8n-seg') #YOLO("tensortt.engine")
model.export(format='engine',imgsz=(720,1280),half=True,simplify=True,device="0")
#outs = model.predict(source="0", show=True)

print('hey')