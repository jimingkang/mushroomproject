import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from hitbot_msgs.srv import SetGPIO

class HitbotControl(Node):
    def __init__(self):
        super().__init__('hitbot_control_pub')

        self.hitbot_x_publisher = self.create_publisher(Int64, '/hitbot_x', 10)
        self.hitbot_y_publisher = self.create_publisher(Int64, '/hitbot_y', 10)
        self.hitbot_z_publisher = self.create_publisher(Int64, '/hitbot_z', 10)
        self.hitbot_r_publisher = self.create_publisher(Int64, '/hitbot_r', 10)

        self.set_gpio_client = self.create_client(SetGPIO, 'set_gpio')
        while not self.set_gpio_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GPIO service not available, waiting again...')

    def publish_hitbot_x(self, data):
        msg = Int64()
        msg.data = data
        self.hitbot_x_publisher.publish(msg)

    def publish_hitbot_y(self, data):
        msg = Int64()
        msg.data = data
        self.hitbot_y_publisher.publish(msg)

    def publish_hitbot_z(self, data):
        msg = Int64()
        msg.data = data
        self.hitbot_z_publisher.publish(msg)

    def publish_hitbot_r(self, data):
        msg = Int64()
        msg.data = data
        self.hitbot_r_publisher.publish(msg)

    def set_gpio(self, gpio_number, set_on):
        req = SetGPIO.Request()
        req.gpio_number = gpio_number
        req.set_on = set_on

        future = self.set_gpio_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('GPIO setting response: %r' % future.result().success)
        else:
            self.get_logger().error('Service call failed to complete')

def main(args=None):
    rclpy.init(args=args)

    hitbot_publisher = HitbotControl()

    try:    
        while rclpy.ok():
            hitbot_x_data = int(input("Enter hitbot_x (mm): "))
            hitbot_y_data = int(input("Enter hitbot_y (mm): "))
            hitbot_z_data = int(input("Enter hitbot_z -(mm): "))
            hitbot_r_data = int(input("Enter hitbot_r (degree): "))
            gpio_number = int(input("Enter GPIO number: "))
            set_on = int(input("Enter set_on (0 = OFF / 1 == ON): "))

            hitbot_publisher.publish_hitbot_x(hitbot_x_data)
            hitbot_publisher.publish_hitbot_y(hitbot_y_data)
            hitbot_publisher.publish_hitbot_z(hitbot_z_data)
            hitbot_publisher.publish_hitbot_r(hitbot_r_data)
            hitbot_publisher.set_gpio(gpio_number, set_on)

    except KeyboardInterrupt:
        pass

    hitbot_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()