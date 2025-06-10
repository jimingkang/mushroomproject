from collections import defaultdict

import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors

track_history = defaultdict(lambda: [])

model = YOLO("./yolo11_x_mushroom_ncnn_model")  # segmentation model
cap = cv2.VideoCapture(0)
w, h, fps = (int(cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))

#out = cv2.VideoWriter("instance-segmentation-object-tracking.avi", cv2.VideoWriter_fourcc(*"MJPG"), fps, (w, h))

while True:
    ret, im0 = cap.read()
    if not ret:
        print("Video frame is empty or video processing has been successfully completed.")
        break

    annotator = Annotator(im0, line_width=2)

    results = model.track(im0, persist=True)

    for result in results:
        boxes = result.boxes  # Boxes object for bounding box outputs
        cls=result.boxes.cls

        masks = result.masks  # Masks object for segmentation masks outputs
        keypoints = result.keypoints  # Keypoints object for pose outputs
        probs = result.probs  # Probs object for classification outputs
        obb = result.obb  # Oriented boxes object for OBB outputs
        #result.show()  # display to screen
        #result.save(filename="result.jpg")  # save to disk
        for box in result.boxes:
            left, top, right, bottom = np.array(box.xyxy.cpu(), dtype=int).squeeze()
            width = right - left
            height = bottom - top
            center = (left + int((right - left) / 2), top + int((bottom - top) / 2))
            label = results[0].names[int(box.cls)]
            confidence = float(box.conf.cpu())
            label= label+",conf:"+str(confidence)
            #if(label=="laptop"):
            cv2.rectangle(im0, (left, top), (right, bottom), (255, 0, 0), 2)
            cv2.putText(im0, label, (left, top - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.imshow("detect", im0)

    #out.write(im0)


    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

#out.release()
cap.release()
cv2.destroyAllWindows()
