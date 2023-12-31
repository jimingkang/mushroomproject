#!/usr/bin/env python
from importlib import import_module
import os

import redis
from flask import Flask, render_template, Response, jsonify

# import camera driver
if os.environ.get('CAMERA'):
    Camera = import_module('camera_' + os.environ['CAMERA']).Camera
else:
    from camera_ipcam import Camera

# Raspberry Pi camera module (requires picamera package)
#from camera_pi import Camera

app = Flask(__name__)
app.config['DEBUG'] = False
#pool = redis.ConnectionPool(host='10.0.0.251', port=6379, decode_responses=True,password='jimmy')
#r = redis.Redis(connection_pool=pool)

@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')
@app.route('/update_mushroom_map' ,methods=['GET'])
def update_mushroom_map():
    detections=r.hgetall("detections")
    print(jsonify({"response":detections}))
    return jsonify({"response":detections})

def gen(camera):
    """Video streaming generator function."""
    yield b'--frame\r\n'
    while True:
        frame = camera.get_frame()
        yield b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n--frame\r\n'


@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', threaded=True,port='5001')
