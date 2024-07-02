from flask import Flask, Response
import pyrealsense2 as rs
import numpy as np
import cv2

class Camera:
    def __init__(self):
        # Initialize the RealSense camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        
        
        # Setup Flask app
        self.app = Flask(__name__)
        self.app.add_url_rule('/', 'index', self.stream_view)
        

    def stream_raw(self):
        # Generator function to stream camera feed
        try:
            profile = self.pipeline.start(self.config)
            print
        except:
            pass
        while True:
            
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert the color frame to a NumPy array
            frame_data = np.array(color_frame.get_data())

            # Encode the frame as JPEG before streaming
            _, buffer = cv2.imencode('.jpg', frame_data)

            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

    def stream_view(self):
        # Flask view to stream the camera feed
        return Response(self.stream_raw(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def start_server(self, port=8000):
        # Method to start the Flask server
        self.app.run(debug=True, host='0.0.0.0', port=port, threaded=True)

if __name__ == '__main__':
    camera = Camera()
    camera.start_server()
