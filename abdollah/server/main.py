from flask import Flask, render_template, request, Response
from config import Config
from view import View
from camera import Camera
from hitbot.HitbotInterface import HitbotInterface


class Robot():
    def __init__(self) -> None:
        self.app = Flask(__name__,static_folder="")
        self.app.add_url_rule('/move/<string:direction>', 'move', self.move)
        self.config = Config()
        self.camera = Camera()
        self.view = View(app = self.app)
        self.initiate_robot()
        self.start_server()
        

    def initiate_robot(self):
        hi=HitbotInterface(92); #//92 is robotid? yes
        hi.net_port_initial()
        ret=hi.initial(1,210); #// I add you on wechat
        print(hi.is_connect())
        print(hi.unlock_position())
        ret = hi.movej_angle(0,0,0,self.config.degree_offset,self.config.movement_speed,0)
        hi.wait_stop()
        hi.get_scara_param()
        print("# Ret:", ret)
        print("# Current:", hi.x,hi.y,hi.z)
        self.hi = hi

    def get_camera(self):
        return self.camera

    def start_server(self, port = 5000):
        self.app.run(debug=True, host='0.0.0.0', port=port)

    def move(self, direction):
        # Open the serial port
        # ser = serial.Serial("/dev/ttyACM0", 115200, timeout=5)
        if direction == "home":
            self.hi.movej_angle(0,0,0,self.config.degree_offset, self.config.movement_speed, 0)
            self.hi.wait_stop()
            return "home"
        elif direction == "custom":
            x = float(request.args.get('x', self.config.default_step))
            y = float(request.args.get('y', self.config.default_step))
            z = float(request.args.get('z', self.config.default_step))
            r = float(request.args.get('r', self.config.default_step))
            self.hi.get_scara_param()
            rett = self.hi.movel_xyz(x, y, z, self.config.degree_offset, self.config.movement_speed)
            self.hi.wait_stop()

            return f"{rett}<br>x = {self.hi.x} {type(x)}<br>y = {self.hi.y} {type(y)}<br>z = {self.hi.z} {type(z)}"
        else:
            amount = float(request.args.get('amount', self.config.default_step))
            x = 0
            y = 0
            z = 0

            if direction == "right":
                x = amount
            elif direction == "left":
                x = -amount
            elif direction == "up":
                y = amount
            elif direction == "down":
                y = -amount
            elif direction == "top":
                z = amount
            elif direction == "bottom":
                z = -amount

            self.hi.get_scara_param()
            rett = self.hi.movel_xyz(self.hi.x + x, self.hi.y + y, self.hi.z + z, self.config.degree_offset, self.config.movement_speed)
            self.hi.wait_stop()

            return f"{rett}\nx = {x}\ny = {y}\nz = {z}"


if __name__ == "__main__":
    Robot()