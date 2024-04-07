from flask import Flask, render_template, Response
from config import Config
from camera import Camera

class View:
    def __init__(self, app) -> None:
        self.app = app
        self.register_routes()

    def register_routes(self):
        # Use the app object to add routes
        self.app.add_url_rule('/', view_func=self.index)
        self.app.add_url_rule('/raw_camera_feed', view_func=self.raw_camera_feed)
        

    def index(self):
        return render_template('index.html',active_page='home')
    
    def raw_camera_feed(self):
        # Assume stream_address is defined somewhere
        camera_port = Config.camera_port
        return render_template('raw_camera_feed.html', camera_port=camera_port, active_page='raw_camera_feed')

    def stream_raw(self):
        return Response(self.camera.stream_raw(), mimetype='multipart/x-mixed-replace; boundary=frame')