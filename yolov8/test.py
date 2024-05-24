from ultralytics import YOLO
import cv2
from ultralytics.utils.plotting import Annotator  # ultralytics.yolo.utils.plotting is deprecated

model = YOLO('best_v2.engine',task="segment")
cap = cv2.VideoCapture(0)
cap.set(3, 736)
cap.set(4, 1280)

while True:
    _, frame = cap.read()

    #img = cv2.cvtColor(frame, (255,0,0))

    results = model.predict(frame,(736, 1280))

    for r in results:

        annotator = Annotator(frame,)

        boxes = r.boxes
        for box in boxes:
            b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
            c = box.cls
            annotator.box_label(b, model.names[int(c)])

    frame = annotator.result()
    cv2.imshow('YOLO V8 Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break

cap.release()
cv2.destroyAllWindows()