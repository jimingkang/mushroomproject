import rclpy
from rclpy.node import Node
from robot_interfaces.srv import RobotJoint

class MushroomGo(Node):
    def __init__(self):
        super().__init__('mushroom_track_node')
        self.point=[]
        self.count=0

        self.cli = self.create_client(RobotJoint, '/move_robot')
        self.angle1=0
        self.angle2=30
        self.angle3=30
        self.req = RobotJoint.Request()

    
    def go(self):
        req = RobotJoint.Request()
        req.angle1 = self.angle1
        req.angle2 = self.angle2
        req.angle3 = self.angle3

        print("Calling service (blocking)...")
        future = self.cli.call_async(req)
        
        rclpy.spin_until_future_complete(self, future)

            


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MushroomGo()
    minimal_publisher.go()
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()