import os
import random
from yolov8 import YOLOv8
import cv2
from ultralytics import YOLO

from tracker import Tracker


video_path = os.path.join('.', 'data', 'people.mp4')
video_out_path = os.path.join('.', 'out.mp4')

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1280)
ret, frame = cap.read()
print(cap.get(3)) 
print(cap.get(2)) 
cap_out = cv2.VideoWriter(video_out_path, cv2.VideoWriter_fourcc(*'MP4V'), cap.get(cv2.CAP_PROP_FPS),
                          (frame.shape[1], frame.shape[0]))

#model = YOLO("models/mush-yolov8-x.onnx")
#model = YOLO("models/yolov8x-seg.onnx")
model = YOLO("weights/mush-yolov8-seg.onnx")
#model= YOLOv8(("mushroomv8.pt", conf_thres=0.5, iou_thres=0.5)

#model_path = "models/yolov8n.onnx"
#yolov8_detector = YOLOv8(model_path, conf_thres=0.5, iou_thres=0.5)
tracker = Tracker()

colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for j in range(10)]

detection_threshold = 0.5
while ret:

    results = model(frame)

    for result in results:
        detections = []
        for r in result.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = r
            x1 = int(x1)
            x2 = int(x2)
            y1 = int(y1)
            y2 = int(y2)
            class_id = int(class_id)
            if score > detection_threshold:
                detections.append([x1, y1, x2, y2, score])
        print(detections)
        if len(detections)>1 :
            tracker.update(frame, detections)
            for track in tracker.tracks:
                bbox = track.bbox
                x1, y1, x2, y2 = bbox
                track_id = track.track_id
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (colors[track_id % len(colors)]), 3)

    cap_out.write(frame)
    #cv2.imshow("frame",frame)
    cv2.waitKey(20)
    ret, frame = cap.read()

cap.release()
cap_out.release()
cv2.destroyAllWindows()
