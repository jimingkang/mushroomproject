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
import numpy as np
import fcntl
import xyz_publish
from tracker import Tracker
__all__ = ["vis"]

from paho.mqtt import client as mqtt_client

broker = '10.0.0.134'
port = 1883
topic = "/flask/scan"
topic4 = "/flask/downmove"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'

tracker = Tracker()
colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for j in range(80)]
global_detections=[]

count=0
def takeSecond(elem):
    return math.sqrt((elem[1]-elem[3])*(elem[1]-elem[3])+(elem[2]-elem[4])*(elem[2]-elem[4]))

cord_x=0
cord_y=0
def vis(img, boxes, scores, cls_ids,conf=0.5, class_names=None):
    global count
    count=(count+1)%100
    print(count)
    val = ''
    file_lock = open("ret.txt", "r")
    try:
        fcntl.flock(file_lock.fileno(), fcntl.LOCK_SH | fcntl.LOCK_NB)
        val = file_lock.read()
        #print(val)
    except IOError:
        print("File is already locked by another process")
    finally:
        # Unlock the file (fcntl.F_UNLOCK)
        fcntl.flock(file_lock.fileno(), fcntl.LOCK_UN)
        # print("File is unlocked")
        file_lock.close()
    detections=[]
    min_distance=[10000000 ,-1,-1,-1,-1,-1]
    for i in range(len(boxes)):
        box = boxes[i]
        cls_id = int(cls_ids[i])
        score = scores[i]
        if score < conf:
            continue
        x0 = int(box[0])
        y0 = int(box[1])
        x1 = int(box[2])
        y1 = int(box[3])
        if score > 0.6 and min_distance[0]>math.sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0)):
            min_distance[0]=math.sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0))
            min_distance[1:5]=score,x0,y0,x1,y1,
            detections.append([x0,y0,x1,y1,score])
    detections.sort(key=takeSecond)
    #print(detections,min_distance)
    new_detections=[]
    if len(detections)>0:
        new_detections.append(detections[0])
        tracker.update(img, new_detections)
        for track in tracker.tracks:

            bbox = track.bbox
            x1, y1, x2, y2 = bbox
            x1 = int(bbox[0])
            y1 = int(bbox[1])
            x2 = int(bbox[2])
            y2 = int(bbox[3])
            #score=int(bbox[4])
            track_id = track.track_id
            #global_detections.append([x0,y0,x1,y1,score,track_id])
            text = 'track_id:{}  {:.1f}%'.format(track_id % len(colors), score * 100)
            txt_color = (0, 0, 0) if np.mean(colors[track_id % len(colors)]) > 0.5 else (255, 255, 255)
            font = cv2.FONT_HERSHEY_SIMPLEX
            txt_size = cv2.getTextSize(text, font, 0.4, 1)[0]
            txt_bk_color = (_COLORS[cls_id] * 255 * 0.7).astype(np.uint8).tolist()
            cv2.rectangle(img, (x1, y1 + 1), (x1 + txt_size[0] + 1, y1 + int(1.5 * txt_size[1])), txt_bk_color, -1)
            cv2.putText(img, text, (x1, y1 + txt_size[1]), font, 0.4, txt_color, thickness=1)
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (colors[track_id % len(colors)]), 1)
            #ret=move_subcribe.run(val)#mqtt_get_value_blocking()
            #print("box count=", count)
            #if  val =='22':
            global cord_x
            global cord_y
            x=int((x1+x2)/2)
            y=int((y1+y2)/2)
            cord_x=x
            cord_y=y
            if  count<15:
                if count>1 and abs(cord_x- x)+abs(cord_y-y)<10:
                    break
                if count==1:
                    coordx = "" + str(x) + "," + str(y) + "," + str(track_id) + ";"
                    coordx = coordx[0:len(coordx) - 1]
                    xyz_publish.run(coordx)
                    print("coxunt=1 ,coordx=",coordx)
            if val=='22' or  count>15:
                count=0
                print("file val=", val,count)
                #coordx = "" + str(x) + "," + str(y) + "," + str(track_id) + ";"
                #coordx = coordx[0:len(coordx) - 1]
                #file_lock = open("ret.txt", "w")
                #try:
                #    fcntl.flock(file_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                #    print(" write File is locked")
                #    file_lock.write('0')
                #except IOError:
                #    print("File is already locked by another process")
                #finally:
                   # Unlock the file (fcntl.F_UNLOCK)
                #    fcntl.flock(file_lock.fileno(), fcntl.LOCK_UN)
                #    print(" write File is unlocked")
                #    file_lock.close()



        color = (_COLORS[cls_id] * 255).astype(np.uint8).tolist()
        #text = '{}:{:.1f}%'.format(class_names[cls_id], score * 100)
        #txt_color = (0, 0, 0) if np.mean(_COLORS[cls_id]) > 0.5 else (255, 255, 255)
        #font  = cv2.FONT_HERSHEY_SIMPLEX

        #txt_size = cv2.getTextSize(text, font, 0.4, 1)[0]
        #cv2.rectangle(img, (x0, y0), (x1, y1), color, 2)

        #txt_bk_color = (_COLORS[cls_id] * 255 * 0.7).astype(np.uint8).tolist()
        #cv2.rectangle(img,(x0, y0 + 1),(x0 + txt_size[0] + 1, y0 + int(1.5*txt_size[1])),txt_bk_color,-1)
        #cv2.putText(img, text, (x0, y0 + txt_size[1]), font, 0.4, txt_color, thickness=1)

    return img


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
