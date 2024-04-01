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
from hitbot.HitbotInterface import HitbotInterface
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import numpy as np
import math
import matplotlib.patches as patches

rs_pipeline = rs.pipeline()
rs_config = rs.config()
rs_config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
rs_config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

model = YOLO('best_v3.pt',task="detect")

def read_config(filename):
    with open(filename, 'r') as config_file:
        config_data = json.load(config_file)
    return config_data

config = read_config('config.json')
for key, value in config.items():
    globals()[key] = value  

camera_x = camera_y = 0
ds = []
pipeline_started = False

# hi=HitbotInterface(92); #//92 is robotid? yes
# hi.net_port_initial()
# ret=hi.initial(1,210); #// I add you on wechat
# print(hi.is_connect())
# print(hi.unlock_position())
# ret = hi.movej_angle(0,0,0,_degree_offset,_movement_speed,0)
# hi.wait_stop()
# hi.get_scara_param()
# print("# Ret:", ret)
# print("# Current:", hi.x,hi.y,hi.z)

app = Flask(__name__,static_folder="assets")

def generate_raw_frames():
    global rs_config,rs_pipeline
    rs_pipeline.start(rs_config)
    while True:
        frames = rs_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert the color frame to a NumPy array
        frame_data = np.array(color_frame.get_data())

        # Encode the frame as JPEG before streaming
        _, buffer = cv2.imencode('.jpg', frame_data)

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

average = np.array([])
coef = 0

def generate_depth():
    global rs_config,rs_pipeline
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        frame_data = np.array(depth_frame.get_data())
        if coef == 0:
            average = frame_data[:]
        else:
            average = np.sum([average * min([coef,1]), frame_data], axis=0) / min([coef+1,2])
        coef += 1

        # Convert the color frame to a NumPy array
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(average, alpha=0.03), cv2.COLORMAP_JET)
        # Encode the frame as JPEG before streaming
        _, buffer = cv2.imencode('.jpg', depth_colormap)
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
def move(direction,amount=_default_step,custom_x=_movement_step,custom_y=_movement_step):
    global camera_x,camera_y,_movement_speed,_degree_offset,hi
    # Open the serial port
    #ser = serial.Serial("/dev/ttyACM0", 115200, timeout=5)
    if direction == "home":
        hi.movej_angle(0,0,0,_degree_offset, _movement_speed, 0)
        hi.wait_stop()
        return "home"
    elif direction == "custom":
        x = float(request.args.get('x', _default_step))
        y = float(request.args.get('y', _default_step))
        z = float(request.args.get('z', _default_step))
        r = float(request.args.get('r', _degree_offset))
        hi.get_scara_param()
        rett=hi.movel_xyz(x,y,z,_degree_offset,_movement_speed)
        hi.wait_stop()

        return f"{rett}<br>x = {hi.x} {type(x)}<br>y = {hi.y} {type(y)}<br>z = {hi.z} {type(z)}"
    else:
        if "amount" in request.args:
            amount = float(request.args.get('amount', _default_step))
        x=0
        y=0
        z=0

        if direction=="right":
            x=amount
        elif direction=="left":
            x=-amount
        elif direction=="up":
            y=amount
        elif direction=="down":
            y=-amount
        elif direction=="top":
            z=amount
        elif direction=="bottom":
            z=-amount
        hi.get_scara_param()
        rett=hi.movel_xyz(hi.x+x,hi.y+y,hi.z+z,_degree_offset,_movement_speed)
        hi.wait_stop()
        
        return f"{rett}\nx = {x}\ny = {y}\nz = {z}"

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
    global camera_x,camera_y,_movement_speed,_degree_offset,hi, rs_config, rs_pipeline
    
    def scan_half_circle(r, h, w):
        # Calculate the number of vertical rectangles
        vertical_rectangles = math.ceil(r / h)
        
        # Initialize the list to hold the center of each rectangle
        rectangle_centers = []
        
        # Loop through each row of rectangles
        for i in range(vertical_rectangles):
            # Calculate the y-coordinate of the rectangle's center
            y_center = (i + 0.5) * h
            
            # Calculate the maximum x-coordinate (half-width) at this height
            y = r - y_center  # Actual y-coordinate in the circle's coordinate system
            max_x = math.sqrt(r**2 - y**2) if y_center < r else 0
            
            # Calculate the number of horizontal rectangles at this height
            horizontal_rectangles = math.ceil((2 * max_x) / w)
            
            # Calculate the width of each rectangle to fit in the half-circle exactly
            actual_w = (2 * max_x) / horizontal_rectangles if max_x > 0 else w
            
            centers = []
            # Loop through each rectangle in this row
            for j in range(horizontal_rectangles):
                # Calculate the x-coordinate of the rectangle's center
                x_center = (j + 0.5) * actual_w - max_x
                
                # Store the center of this rectangle
                centers.append((x_center, y_center))
            
            rectangle_centers += centers if i%2 else centers[::-1]
        
        return rectangle_centers

    def check_points(points, threshold):
        def distance(point1, point2):
            return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

        unique_points = []
        for point in points:
            if all(distance(point, unique_point) >= threshold for unique_point in unique_points):
                unique_points.append(point)

        return unique_points

    def plot_scan(r, h, w, scanned_success, scanned_fail, center_xs, center_ys):
        # Get the sequence of rectangle centers
        
        fig = Figure()
        canvas = fig.canvas
        ax = fig.gca()
        canvas.draw()
        # Plot the half-circle
        theta = np.linspace(0, np.pi, 100)
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        ax.plot(x, r-y, 'b')  # Half-circle border
        
        i = 0
        # Plot the rectangles
        for center in scanned_success:
            rect = plt.Rectangle((center[0]-w/2, center[1]-h/2), w, h, linewidth=1, edgecolor='g', facecolor='#00FF005F')
            ax.text(center[0], center[1], f'{i+1}', fontsize = 14)
            ax.add_patch(rect)
            i+=1
        
        # Plot the rectangles
        for center in scanned_fail:
            rect = plt.Rectangle((center[0]-w/2, center[1]-h/2), w, h, linewidth=1, edgecolor='r', facecolor='#FF00005F')
            ax.text(center[0], center[1], f'{i+1}', fontsize = 14)
            ax.add_patch(rect)
            i+=1

        ax.plot(center_xs,center_ys,'ro')

        
        # Adjust plot limits and aspect ratio
        ax.set_xlim(-r*1.5, r*1.5)
        ax.set_ylim(-r/2, r*1.5)
        ax.set_aspect('equal', adjustable='box')
        fig.savefig("scan.png")

    def capture_annotate(i):
        global rs_pipeline, rs_config, model
        rs_pipeline.start(rs_config)
        frames = rs_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        # Convert the color frame to a NumPy array
        frame_data = np.array(color_frame.get_data())
        rs_pipeline.stop()
        results = model.predict(frame_data, stream=False, imgsz=(736,1280),conf=0.9)
        centers = []
        for r in results:
            # Convert the color frame to a NumPy array
            annotated_data = r.plot()
            boxes = r.boxes.xyxyn.cpu().numpy()
            centers += [((box[0]+box[2]) / 2 - 0.5, (box[1]+box[3]) / 2 - 0.5) for box in boxes]
            cv2.imwrite(f"annotated.jpg",annotated_data)
        return centers
    # Example parameters
    hi.get_scara_param()
    r = 60.0  # Radius of the half-circle
    w = math.tan(87/2 * math.pi / 180.0) * (29 + (hi.z/10)) * 2  # Height of the rectangles
    h = math.tan(58/2 * math.pi / 180.0) * (29 + (hi.z/10)) * 2 
    print(f"W x H = ({w} x {h})")
    rectangle_centers = scan_half_circle(r, h * 0.5, w * 0.5)
    scanned_success = []
    scanned_fail = []
    mushrooms_centers_xs = []
    mushrooms_centers_ys = []
    for i,c in enumerate(rectangle_centers):
        y , x = c[0] * 10, c[1] * 10  
        x_target = 600 - x
        y_target = y
        print(x_target, y_target)
        ret=hi.new_movej_xyz_lr(x_target, y_target, hi.z, _degree_offset, _movement_speed, 0, -1)
        print(ret)
        hi.wait_stop()
        if ret == 7:
            hi.movej_angle(0,0,hi.z,_degree_offset,_movement_speed*2,0)
            hi.wait_stop()
            #ret=hi.movel_xyz(x_target / 2, y_target / 2, hi.z, _degree_offset, _movement_speed)
            ret=hi.new_movej_xyz_lr(x_target, y_target, hi.z, _degree_offset, _movement_speed, 0, 1)
            hi.wait_stop()
            if ret != 1:
                scanned_fail += [c]
            else:
                scanned_success += [c]
        else:
            scanned_success += [c]
        
        mushroom_centers = capture_annotate(i)
        current_x, current_y = c
        center_xs = []
        center_ys = []
        for center in mushroom_centers:
            x,y = center
            center_xs += [current_x + x * w]
            center_ys += [current_y - y * h]
        mushrooms_centers_xs += center_xs
        mushrooms_centers_ys += center_ys
        mushrooms_centers = check_points(list(zip(mushrooms_centers_xs, mushrooms_centers_ys)), 10)
        mushrooms_centers = list(zip(*mushrooms_centers))
        mushrooms_centers_xs, mushrooms_centers_ys = list(mushrooms_centers[0]), list(mushrooms_centers[1])
        plot_scan(r,h,w,scanned_success,scanned_fail, mushrooms_centers_xs, mushrooms_centers_ys)
        


    return Response("Done")

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
    app.run(debug=True, host='0.0.0.0', port=5001)

    
