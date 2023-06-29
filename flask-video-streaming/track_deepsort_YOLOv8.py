import os
import random
from yolov8 import YOLOv8
import cv2
from ultralytics import YOLO

from tracker import Tracker


video_path = os.path.join('.', 'data', 'people.mp4')
video_out_path = os.path.join('.', 'out.mp4')

cap = cv2.VideoCapture("http://10.0.0.251:5000/video_feed")
ret, frame = cap.read()


model_path = "models/mush-yolov8-x.onnx"
#model_path = "models/best.onnx"
#model_path = "models/yolov8x-seg.onnx"
yolov8_detector = YOLOv8(model_path, conf_thres=0.5, iou_thres=0.5)
tracker = Tracker()

colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for j in range(10)]
detection_threshold = 0.5
while ret:

    (results ,scores,class_ids)= yolov8_detector(frame)
    i=0
    for result in results:
        detections = []
        if len(result)>0 :
            #for r in result:
            #x1, y1, x2, y2 = result
            x1 = int(result[0])
            x2 = int(result[1])
            y1 = int(result[2])
            y2 = int(result[3])
            score=scores[i]
            class_id=class_ids[i]
            class_id = int(class_id)
            i=i+1
            if score > detection_threshold:
                detections.append([x1, y1, x2, y2, score])
            print(detections)
            #if len(detections)>0 :
            #    tracker.update(frame, detections)
            #    for track in tracker.tracks:
            #        bbox = track.bbox
            #        x1, y1, x2, y2 = bbox
            #        track_id = track.track_id
            #        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (colors[track_id % len(colors)]), 3)

    #cap_out.write(frame)
    combined_img = yolov8_detector.draw_detections(frame)
    #cv2.imshow("Detected Objects", combined_img)
    cv2.imshow("frame",combined_img)
    cv2.waitKey(20)
    ret, frame = cap.read()

cap.release()
#cap_out.release()
cv2.destroyAllWindows()
