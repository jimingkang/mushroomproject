import cv2
import numpy as np
import pyrealsense2 as rs
from flask import Flask, Response

app = Flask(__name__)

# Initialize the RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

pipeline.start(config)

def generate_frames():
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

@app.route('/')
def index():
    return "RealSense Camera Streaming Server"

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/annotate_stream')
def annotate_stream():
    return Response(annotate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
