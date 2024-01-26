from flask import Flask, render_template, request, Response
import serial
import time
import requests
import json
from collections import Counter
import numpy as np
import math
import cv2
from ultralytics import YOLO
import urllib
import pyrealsense2 as rs

camera_x = camera_y = 0
ds = []
pipeline_started = False

app = Flask(__name__)

def read_config(filename):
    with open(filename, 'r') as config_file:
        config_data = json.load(config_file)
    return config_data
config = read_config('config.json')
for key, value in config.items():
    globals()[key] = value

ser = serial.Serial("/dev/ttyACM0",115200)
def command(ser, command):
    if command:  # checks if string is empty
        print("Sending gcode:" + str(command))
        command = str.encode(command)
        ser.write(command)  # Send g-code
        return 'ok'

def generate_raw_frames():

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert the color frame to a NumPy array
        frame_data = np.array(color_frame.get_data())

        # Encode the frame as JPEG before streaming
        _, buffer = cv2.imencode('.jpg', frame_data)

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

def generate_depth():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    pipeline.start(config)
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        # Convert the color frame to a NumPy array
        frame_data = np.array(depth_frame.get_data())
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(frame_data, alpha=0.03), cv2.COLORMAP_JET)
        # Encode the frame as JPEG before streaming
        _, buffer = cv2.imencode('.jpg', depth_colormap)
        print(depth_colormap.shape)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/annotated_camera_feed')
def annotated_camera_feed():
    global _movement_step
    return render_template('annotated_camera_feed.html',_movement_step=_movement_step)

@app.route('/depth_feed')
def depth_feed():
    return render_template('depth_feed.html')

@app.route('/automatic_scan')
def automatic_scan():
    return render_template('automatic_scan.html')

@app.route('/raw_camera_feed')
def raw_camera_feed():
    return render_template('raw_camera_feed.html', stream_address=stream_address)

@app.route('/move/<string:direction>')
def move(direction,amount=_movement_step,custom_x=_movement_step,custom_y=_movement_step):
    global camera_x,camera_y,_movement_step,_feedback
    # Open the serial port
    #ser = serial.Serial("/dev/ttyACM0", 115200, timeout=5)
    if direction == "home":
        command(ser, "$X\r\n")
        time.sleep(1)
        command(ser, "$H\r\n")
        camera_x = camera_y = 0
        return "home"
    elif direction == "custom":
        command(ser, "$X\r\n")
        time.sleep(1)
        command(ser, "$H\r\n")
        camera_x = camera_y = 0

        time.sleep(7)

        x = float(request.args.get('x', custom_x))
        y = float(request.args.get('y', custom_y))
        cmd =f'G91 G21 G1 X{x} Y{y}'
        command(ser, f"{cmd} F{_feedback}\r\n")
        camera_x += x
        camera_y += y
        return cmd
    else:
        if "amount" in request.args:
            amount = float(request.args.get('amount', _movement_step))
        x=0
        y=0
        if direction=="right":
            x=-amount
        elif direction=="left":
            x=amount
        elif direction=="up":
            y=amount
        elif direction=="down":
            y=-amount
        cmd =f'G91 G21 G1 X{x} Y{y}'
        command(ser, f"{cmd} F{_feedback}\r\n")
        camera_x += x
        camera_y += y
        return f"{cmd} F{_feedback}\r\n"

def find_common_points(frames, threshold, n_common_points):
    # Flatten the frames into 2D array of points
    all_points = frames#.reshape(-1, frames.shape[-1])

    # Initialize counter for each point
    counters = np.zeros(len(all_points))

    # Calculate pairwise distance and update counters
    for i in range(len(all_points)):
        for j in range(i+1, len(all_points)):
            dist = np.linalg.norm(all_points[i]-all_points[j])
            if dist < threshold:
                counters[i] += 1
                counters[j] += 1

    # Find n_common_points most common points
    common_points = all_points[np.argsort(-counters)][:n_common_points]

    return common_points

def get_filtered_centers():
    url = "http://172.26.49.56:5000/annotate_frame"
    response = requests.get(url)
    l = json.loads(response.text)
    second_dim_lengths = [len(sublist) for sublist in l]
    most_repeated_length = Counter(second_dim_lengths).most_common(1)[0][0]
    centers = np.array([[(x0 + x1) / 2, (y0 + y1) / 2] for r in l for x0, y0, x1, y1,_,_ in r])
    filtered_centers = find_common_points(centers,100,most_repeated_length)
    return filtered_centers.tolist()

def px2mm(x,y,cx,cy):
    coef = 265/720

    x_coord_coef = 25
    y_coord_coef = 13

    #translate px coords to camera center = 0,0
    x -= (1280)/2
    y -= (720)/2

    x = -x
    y = -y

    print("x:",x,"y:",y)
    x_mm = x*coef
    y_mm = y*coef

    print("x_mm:", x_mm, "y_mm:",y_mm)
    cx_mm = cx*x_coord_coef
    cy_mm = cy*y_coord_coef
    print("cx_mm:", cx_mm, "cy_mm:",cy_mm)
    g_x_mm = cx_mm + x_mm
    g_y_mm = cy_mm + y_mm

    print("global", g_x_mm,g_y_mm)
    return g_x_mm,g_y_mm

def mm2coord(x_mm,y_mm):
    x_coord_coef = 25
    y_coord_coef = 13
    x_coord = x_mm / x_coord_coef
    y_coord = y_mm / y_coord_coef
    return x_coord,y_coord

def check_points(points, threshold):
    def distance(point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    unique_points = []
    for point in points:
        if all(distance(point, unique_point) >= threshold for unique_point in unique_points):
            unique_points.append(point)

    return unique_points

def process_ds():
    global ds
    centers_mm = []
    for row in ds:
        centers = row["centers"]
        camera_x = row["camera_x"]
        camera_y = row["camera_y"]
        for c in centers:
            c_mm = px2mm(c[0],c[1],camera_x,camera_y)
            centers_mm += [c_mm]
    
    unique_points = check_points(centers_mm,10)
    return unique_points

@app.route('/scan')
def scan():
    global ds
    move("custom",custom_x=-7,custom_y=-3)
    time.sleep(13)
    
    filtered_centers = get_filtered_centers()
    ds += [{
        "centers": filtered_centers,
        "camera_x": camera_x,
        "camera_y": camera_y
    }]

    move("down",10)
    time.sleep(5)

    filtered_centers = get_filtered_centers()
    ds += [{
        "centers": filtered_centers,
        "camera_x": camera_x,
        "camera_y": camera_y
    }]

    move("down",15)
    time.sleep(5)

    filtered_centers = get_filtered_centers()
    ds += [{
        "centers": filtered_centers,
        "camera_x": camera_x,
        "camera_y": camera_y
    }]

    # move("down",10)
    # time.sleep(5)

    # filtered_centers = get_filtered_centers()
    # ds += [{
    #     "centers": filtered_centers,
    #     "camera_x": camera_x,
    #     "camera_y": camera_y
    # }]

    # move("down",10)
    # time.sleep(5)

    # filtered_centers = get_filtered_centers()
    # ds += [{
    #     "centers": filtered_centers,
    #     "camera_x": camera_x,
    #     "camera_y": camera_y
    # }]

    unique_points = process_ds()

    coords = []
    for c_mm in unique_points:
        x,y = c_mm
        coords += [[mm2coord(x,y)]]
    return Response(json.dumps(coords), content_type='application/json')

def generate_annotated_frames():
    global stream_address
    model = YOLO(stream_address,'best_v3.engine',task="segment")
    results = model("", stream=True, imgsz=(736,1280),conf=0.95)
    for r in results:

        # Convert the color frame to a NumPy array
        frame_data = r.plot()
        
        # Encode the frame as JPEG before streaming
        _, buffer = cv2.imencode('.jpg', frame_data)
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

def annotate_frames():
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert the color frame to a NumPy array
        frame_data = np.array(color_frame.get_data())

        # Encode the frame as JPEG before streaming
        _, buffer = cv2.imencode('.jpg', frame_data)

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/annotate_stream')
def annotate_stream():
    return Response(generate_annotated_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/annotate_frame')
def annotate_frame():
    model = YOLO('best_v3.engine',task="segment")
    results = model(stream_address, stream=True, imgsz=(736,1280),conf=0.95)
    l = []
    for r in results:
        l += [r.boxes.data.tolist()]
        if len(l) >= 6:
            return Response(json.dumps(l), content_type='application/json')

@app.route('/stream_raw')
def stream_raw():
    return Response(generate_raw_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stream_depth')
def stream_depth():
    return Response(generate_depth(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')

    
