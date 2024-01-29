import io
import random
import redis
from ultralytics import YOLO
from base_camera import BaseCamera
import cv2
from tracker import Tracker
#import move_subcribe
import xyz_publish
#from ultralytics.utils.plotting import Annotator

from paho.mqtt import client as mqtt_client

count = 0
ip=''
broker=''
redis_server=''
try:
    for line in open("../ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:len(line)-1]
        if line[0:6] == "reddis":
            redis_server=line[9:len(line)-1]
except:
    pass
print(broker)
print(redis_server)
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)

#broker = '192.168.254.42'
#broker = '10.0.0.134'
port = 1883
topic = "/flask/xyz"
topic4 = "/flask/downmove"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'

IMAGE_EXT = [".jpg", ".jpeg", ".webp", ".bmp", ".png"]
model = YOLO('best_v3.engine',task="segment")  # pretrained YOLOv8n model

tracker = Tracker()


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            i=1
            #print("xyx publish Connected to MQTT Broker!")
        else:
            i=0
            #print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def takeSecond(ele):
    return abs(ele[0]-ele[2])

class Camera(BaseCamera):
    """Requires python-v4l2capture module: https://github.com/gebart/python-v4l2capture"""

    video_source = "/dev/video0"
    @staticmethod
    def frames():
        #video = cv2.VideoCapture(video_source)
        video = cv2.VideoCapture("http://192.168.0.100:5000/video_feed")
        #video = cv2.VideoCapture("http://192.168.1.3:5000/video_feed")
        size_x =1280
        size_y = 720
        video.set(cv2.CAP_PROP_FRAME_WIDTH, size_x)
        video.set(cv2.CAP_PROP_FRAME_HEIGHT, size_y)
        video.set(cv2.CAP_PROP_FPS, 30)


        try:
            i=1

            while video.isOpened():
                i = i + 1
                ret, img = video.read()

                if not ret:
                    break
                global count
                count = (count + 1) % 1000

                #if len(r.hkeys("detections"))<=0 and ret:
                if r.get("mode")=="camera_ready" and ret:
                #if  ret:
                    results = model.predict(img, imgsz=(736, 1280))  # return a generator of Results objects
                    # Process results generator
                    for result in results:
                        print(result)

                        boxes = result.boxes  # Boxes object for bbox outputs
                        xyxy=result.boxes.xyxy  # box with xyxy format, (N, 4)
                        xyxy_list=xyxy.tolist()
                        xywh=result.boxes.xywh  # box with xywh format, (N, 4)
                        masks = result.masks  # Masks object for segmentation masks outputs
                        keypoints = result.keypoints  # Keypoints object for pose outputs
                        conf = result.boxes.conf.tolist()  # Probs object for classification outputs
                        detections = []
                        for i  in  range(len(xyxy_list)):
                            xyz=xyxy_list[i]
                            x0 = int(xyz[0])
                            y0 = int(xyz[1])
                            x1 = int(xyz[2])
                            y1 = int(xyz[3])
                            if (conf[i]*100)>50 and (abs(x1-x0)>5 and abs(x1-x0)<200) and (abs(y1-y0)<200 and abs(y0-y1)>5) and (abs(abs(x1-x0)-abs(y1-y0))<10):
                                detections.append([x0, y0, x1, y1, conf[i]*100])
                        detections.sort(key=takeSecond,reverse=True)
                        new_detections=[]
                        coordx = ""
                        if len(detections) > 0:
                            new_detections.append(detections[0])
                            #global pre_tracker
                            #pre_tracker = tracker
                            tracker.update(img, new_detections)
                            for track in tracker.tracks:
                                bbox = track.bbox
                                x1, y1, x2, y2 = bbox
                                x1 = int(bbox[0])
                                y1 = int(bbox[1])
                                x2 = int(bbox[2])
                                y2 = int(bbox[3])
                                # score=int(bbox[4])
                                track_id = track.track_id
                                text = 'id:{},x:{},y:{}'.format(track_id,(x1+x2)/2,(y1+y2)/2)
                                font = cv2.FONT_HERSHEY_SIMPLEX
                                txt_size = cv2.getTextSize(text, font, 0.4, 1)[0]
                                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 1)
                                cv2.putText(img, text, (x1, y1 + txt_size[1]), font, 0.4, (0, 255, 0), thickness=1)
                                #if count>20 and (x2<800 and x1>50) and (y2<450 and y1>50) and (abs(x1-x2)>5 and abs(x1-x2)<200) and (abs(y1-y2)<200 and abs(y2-y1)>5) and abs(abs(x1-x2)-abs(y1-y2))<10 and  ( (int((x1 + x2) / 2)>440 or int((x1 + x2) / 2)<400) or (int((y1 +y2) / 2)>(260+0) or int((y1 + y2) / 2)<(220+0))):
                                if  r.get("mode")=="camera_ready":
                                    count=0
                                    print(" global count=",count)
                                    coordx = "" + str(int((x1 + x2) / 2)) + "," + str(int((y1 + y2) / 2)) + "," + str(track_id) + ";"
                                    coordx = coordx[0:len(coordx) - 1]
                                    print("coordx=",coordx)
                                    xyz_publish.run(coordx)

                                #if   (abs(x1-x2)>5 and abs(x1-x2)<200) and (abs(y1-y2)<200 and abs(y2-y1)>5) and (abs(abs(x1-x2)-abs(y1-y2))<10):
                                #    print("count=",count)
                                #    coordx =coordx+ "" + str(int((x1 + x2) / 2)) + "," + str(int((y1 + y2) / 2)) + "," + str(track_id) + ";"
                                #    coordx = coordx[0:len(coordx) - 1]
                                    #xyz_publish.run(coordx)
                        #if  r.get("mode")=="camera_ready":
                        #    coordx = coordx[0:len(coordx) - 1]
                        #    print("coordx=",coordx)
                        #    xyz_publish.run(coordx)
                        #    r.set("mode","pickup_ready")

                yield cv2.imencode('.jpg', img)[1].tobytes()
        finally:
            video.release()
