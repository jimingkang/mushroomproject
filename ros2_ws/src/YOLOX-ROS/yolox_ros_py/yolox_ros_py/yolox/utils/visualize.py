#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# Copyright (c) Megvii Inc. All rights reserved.
import math
import random
import ssl
import subprocess
import threading
import time

import cv2
from loguru import logger
import numpy as np
import redis

#import fcntl
#import xyz_publish
#import pickup_publish
from yolox_ros_py.tracker import Tracker
__all__ = ["vis"]

from paho.mqtt import client as mqtt_client

broker = '172.27.34.65'
port = 1883
topic = "/flask/scan"
#topic4 = "/flask/downmove"
topic4 = "/flask/pickup"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'

tracker = Tracker()
colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for j in range(80)]

pool = redis.ConnectionPool(host=broker, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)

def takeSecond(elem):
    return math.sqrt((elem[1]-elem[3])*(elem[1]-elem[3])+(elem[2]-elem[4])*(elem[2]-elem[4]))


pre_tracker= Tracker()

focus=1035
ref_real_width=35 #mm
ref_real_distance=125
def focus_length(width_in_pixel,real_distance,real_width):
    return width_in_pixel*real_distance/real_width
def find_distance(focus_length,real_width,width_in_pixel):
    return real_width*focus_length/width_in_pixel
def find_XY(width_in_pixel,real_distance,real_width):
    return width_in_pixel*real_distance/focus
def vis(img, boxes, scores, cls_ids,count,conf=0.3, class_names=None):
    detections = []
    track_ids=[]
    min_distance=[10000000 ,-1,-1,-1,-1,-1]
    for i in range(len(boxes)):
        box = boxes[i]
        cls_id = int(cls_ids[i])
        score = scores[i]
        #if score < conf:
        #    continue
        x0 = int(box[0])
        y0 = int(box[1])
        x1 = int(box[2])
        y1 = int(box[3])

        if score > 0.3 and abs(int(x1-x0)/int(y1-y0))<1.1 and abs(int(x1-x0)/int(y1-y0))>0.9:
            #min_distance[0]=math.sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0))
            #min_distance[1:5]=score,x0,y0,x1,y1,
            detections.append([x0,y0,x1,y1,score])
    #detections.sort(key=takeSecond)
    #logger.info("new local site_packages detections:",detections)
    new_detections=[]
    if len(detections)>0:
        new_detections.append(detections[0])
        global pre_tracker
        pre_tracker=tracker
        tracker.update(img, detections)
        #tracker.update(img, detections)
        coordx =""
        for track in tracker.tracks:

            bbox = track.bbox
            x1, y1, x2, y2 = bbox
            x1 = int(bbox[0])
            y1 = int(bbox[1])
            x2 = int(bbox[2])
            y2 = int(bbox[3])
            #score=int(bbox[4])
            track_id = track.track_id
            track_ids.append(track_id)
            #for pre_track in pre_tracker:
            #    pre_bbox = pre_track.bbox
            #    pre_x1, pre_y1, pre_x2, pre_y2 = pre_bbox
            #    pre_x1 = int(pre_bbox[0])
            #    pre_y1 = int(pre_bbox[1])
            #    pre_x2 = int(pre_bbox[2])
            #    pre_y2 = int(pre_bbox[3])
            #    print("diff x+x2",abs(int((x1 + x2)))-abs(abs(int((x1 + x2)))))
            #    if track_id==pre_track.track_id and abs(int((x1 + x2)))-abs(abs(int((x1 + x2))))>100:
            #        coordx = "" + str(int((x1 + x2) / 2)) + "," + str(int((y1 + y2) / 2)) + "," + str(track_id) + ";"
            #        coordx = coordx[0:len(coordx) - 1]
            #        print("coordx=", coordx)
            #        xyz_publish.run(coordx)






            #distance=find_distance(focus,35,x2-x1)
            #camera_x=find_XY((x2+x1)/2.0,distance,focus)
            #camera_y = find_XY((y2 + y1) / 2.0, distance, focus)

            #global_camera_xy = r.get("global_camera_xy")
            #if global_camera_xy is not None:
            #    camera_x = camera_x + int(float(global_camera_xy))
            #    #distance = int(math.sqrt(camera_x * camera_x + camera_y * camera_y))
            #    if not r.hexists("detections", str(track_id)):
            #        #print("distance:" + str(distance))  # +"camera_x:"+int(float(camera_x)))
            #        obj = str(camera_x) + "," + str(camera_y) + "," + str(track_id) + "," + str(distance)  # Mushroom(math.sqrt(camera_x * camera_x + camera_y * camera_y),track_id,camera_x,camera_y) #
            #        # r.zadd("detections_index", {obj: distance})
            #        r.hmset("detections",
            #                {str(track_id): str(camera_x) + "," + str(camera_y) + "," + str(distance) + "," + str(track_id)})
            #else:
            #    r.set("global_camera_xy",'0')


            text = 'id:{}  {:.1f}%,{},{}'.format(track_id % len(colors), score * 100,int((x1+x2)/2),int((y1+y2)/2))
            txt_color = (0, 0, 0) if np.mean(colors[track_id % len(colors)]) > 0.5 else (255, 255, 255)
            font = cv2.FONT_HERSHEY_SIMPLEX
            txt_size = cv2.getTextSize(text, font, 0.4, 3)[0]
            txt_bk_color = (_COLORS[cls_id] * 255 * 0.7).astype(np.uint8).tolist()
            #cv2.rectangle(img, (x1, y1 + 1), (x1 + txt_size[0] + 1, y1 + int(1.5 * txt_size[1])), txt_bk_color, -1)
            cv2.putText(img, text, (x1, y1 - txt_size[1]), font, 0.4, txt_color, thickness=1)
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (colors[track_id % len(colors)]),2)
            cv2.circle(img, (int((x1+x2)/2), int((y1+y2)/2)), 4, (255, 255, 255), 1)
            #cv2.putText(img, 'xy:{} {} '.format(str((x1+x2)/2), str((y1+y2)/2)), ((x1+x2)/2, (y1+y2)/2), font, 0.4, txt_color, thickness=1)
            #ret=move_subcribe.run(val)#mqtt_get_value_blocking()
            #if count>20  and  ( (int((x1 + x2) / 2)>440 or int((x1 + x2) / 2)<400) or (int((y1 +y2) / 2)>(260+0) or int((y1 + y2) / 2)<(220+0))):






    return img,track_ids


_COLORS = np.array(
    [
        0.000, 0.447, 0.741,
        0.850, 0.325, 0.098,
        0.929, 0.694, 0.125,
        0.494, 0.184, 0.556,
        0.466, 0.674, 0.188,
        0.301, 0.745, 0.933,
        0.635, 0.078, 0.184,
        0.300, 0.300, 0.300,
        0.600, 0.600, 0.600,
        1.000, 0.000, 0.000,
        1.000, 0.500, 0.000,
        0.749, 0.749, 0.000,
        0.000, 1.000, 0.000,
        0.000, 0.000, 1.000,
        0.667, 0.000, 1.000,
        0.333, 0.333, 0.000,
        0.333, 0.667, 0.000,
        0.333, 1.000, 0.000,
        0.667, 0.333, 0.000,
        0.667, 0.667, 0.000,
        0.667, 1.000, 0.000,
        1.000, 0.333, 0.000,
        1.000, 0.667, 0.000,
        1.000, 1.000, 0.000,
        0.000, 0.333, 0.500,
        0.000, 0.667, 0.500,
        0.000, 1.000, 0.500,
        0.333, 0.000, 0.500,
        0.333, 0.333, 0.500,
        0.333, 0.667, 0.500,
        0.333, 1.000, 0.500,
        0.667, 0.000, 0.500,
        0.667, 0.333, 0.500,
        0.667, 0.667, 0.500,
        0.667, 1.000, 0.500,
        1.000, 0.000, 0.500,
        1.000, 0.333, 0.500,
        1.000, 0.667, 0.500,
        1.000, 1.000, 0.500,
        0.000, 0.333, 1.000,
        0.000, 0.667, 1.000,
        0.000, 1.000, 1.000,
        0.333, 0.000, 1.000,
        0.333, 0.333, 1.000,
        0.333, 0.667, 1.000,
        0.333, 1.000, 1.000,
        0.667, 0.000, 1.000,
        0.667, 0.333, 1.000,
        0.667, 0.667, 1.000,
        0.667, 1.000, 1.000,
        1.000, 0.000, 1.000,
        1.000, 0.333, 1.000,
        1.000, 0.667, 1.000,
        0.333, 0.000, 0.000,
        0.500, 0.000, 0.000,
        0.667, 0.000, 0.000,
        0.833, 0.000, 0.000,
        1.000, 0.000, 0.000,
        0.000, 0.167, 0.000,
        0.000, 0.333, 0.000,
        0.000, 0.500, 0.000,
        0.000, 0.667, 0.000,
        0.000, 0.833, 0.000,
        0.000, 1.000, 0.000,
        0.000, 0.000, 0.167,
        0.000, 0.000, 0.333,
        0.000, 0.000, 0.500,
        0.000, 0.000, 0.667,
        0.000, 0.000, 0.833,
        0.000, 0.000, 1.000,
        0.000, 0.000, 0.000,
        0.143, 0.143, 0.143,
        0.286, 0.286, 0.286,
        0.429, 0.429, 0.429,
        0.571, 0.571, 0.571,
        0.714, 0.714, 0.714,
        0.857, 0.857, 0.857,
        0.000, 0.447, 0.741,
        0.314, 0.717, 0.741,
        0.50, 0.5, 0
    ]
).astype(np.float32).reshape(-1, 3)
