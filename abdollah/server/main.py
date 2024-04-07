from flask import Flask, render_template, request, Response
from config import Config
from view import View
from camera import Camera


class Robot():
    def __init__(self) -> None:
        self.app = Flask(__name__,static_folder="")
        self.camera = Camera()
        self.view = View(app = self.app)
        self.start_server()
        

    def get_camera(self):
        return self.camera

    def start_server(self, port = 5000):
        self.app.run(debug=True, host='0.0.0.0', port=port)


if __name__ == "__main__":
    Robot()