import time
import math
from .HitbotInterface import HitbotInterface
import numpy as np

class ScaraRobot(HitbotInterface):
    """
    A wrapper class around HitbotInterface for controlling a SCARA robot.
    Provides convenience methods for initialization, homing, and updates.
    """

    def __init__(self, robot_id=92, port=210):
        super().__init__(robot_id)  # Properly call parent constructor

        # Initialize communication and connect
        self.net_port_initial()
        ret = self.initial(1, port)
        
        # Verify connection
        if not self.is_connect():
            raise ConnectionError("Failed to connect to SCARA robot.")

        # Unlock and retrieve parameters
        self.unlock_position()
        self.get_scara_param()

        print("Robot ready.")

        # Initialize internal state variables
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.angle3 = 0.0
        self.r = 0.0
        self.limit_degree=[[-90,90],[-120,120],[-120,120]]
        self.go_home()
    
    def within_limits(self,angle1,angle2,angle3):
        angles = [angle1, angle2, angle3]
        for i, angle in enumerate(angles):
            lower, upper = self.limit_degree[i]
            if not (lower <= angle <= upper):
                return False
        return True

    def go_home(self, speed=50, roughly=0):
        """
        Moves the robot to its home (zero) position.
        """
        self.unlock_position()
        
        ret = self.movej_angle(
            goal_angle1=0,
            goal_angle2=0,
            goal_z=0,
            goal_r=0,
            speed=speed,
            roughly=roughly
        )
        if ret == 1:
            print("Robot homed successfully.")
        else:
            print(f"Failed to move home. Error code: {ret}")
        self.wait_stop()
        return ret

    def update(self):
        """
        Fetch the current robot parameters (angles, position, etc.)
        and update internal state variables.
        """
        self.get_scara_param()
        self.angle3=self.r-self.angle1-self.angle2
    
    def move_joint_degree(self,theta1,theta2,theta3,speed=50, roughly=0):
        
        self.get_scara_param()
        angle3=theta1+theta2+theta3
        print(f"before conversion angle3: {angle3}")
        #angle3 = (angle3 + 180) % 360 - 180  #0-360 to -180~180
        print(f"after conversion angle3: {angle3}")
        if not self.within_limits(theta1,theta2,theta3):
            print("out of limit")
            return 0
    
        ret = self.movej_angle(
            goal_angle1=theta1,
            goal_angle2=theta2,
            goal_z=self.z,
            goal_r=angle3,
            speed=speed,
            roughly=roughly
        )
        self.wait_stop()
        self.update()
        return ret

    def move_joint_radian(self,theta1,theta2,theta3,speed=50, roughly=0):
        theta1=int(round(np.rad2deg(theta1),0))
        theta2=int(round(np.rad2deg(theta2),0))
        theta3=int(round(np.rad2deg(theta3),0))

        self.move_joint_degree(theta1,theta2,theta3,speed, roughly)
    
    def move_z(self,z,speed=50, roughly=0):
        self.get_scara_param()
        if self.z>-170:
            ret = self.movej_angle(
                goal_angle1=self.angle1,
                goal_angle2=self.angle2,
                goal_z=z,
                goal_r=self.r,
                speed=speed,
                roughly=roughly
            )
            self.wait_stop()
        else:
            print("z out of range")
if __name__=="__main__":
    scara=ScaraRobot()
    scara.go_home()
    #scara.move_joint_radian(0.1,0.1,0.1,70)
    #scara.move_z(-10)