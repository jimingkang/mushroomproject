import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, Float32
import time
from scipy.differentiate import jacobian
from scipy.linalg import pinv
import math
from std_srvs.srv import Trigger
import numpy




def find_and_draw_templates(main_image, templates, threshold=0.7):
    """
    Finds templates in the main image and draws rectangles around them.

    Args:
        main_image_path (str): Path to the main image.
        template_paths (list of str): List of paths to template images.
        threshold (float): Matching threshold (default 0.8).

    Returns:
        image (numpy.ndarray): Main image with rectangles drawn around matches.
    """
    # Load main image and convert to grayscale
    main_gray = cv2.cvtColor(main_image, cv2.COLOR_BGR2GRAY)
    w_main, h_main = main_gray.shape[::-1]
    # Colors to use for rectangles (you can add more if needed)
    colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (0, 255, 255)]
    
    points=[]
    for idx, template in enumerate(templates):
        
        template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        w, h = template_gray.shape[::-1]

        # Match template
        res = cv2.matchTemplate(main_gray, template_gray, cv2.TM_CCOEFF_NORMED)
        loc = np.where(res >= threshold)

        # Draw rectangles for all matches found
        color = colors[idx % len(colors)]
        
        for pt in zip(*loc[::-1]):
            points.append([(pt[0] + w/2)/w_main, (pt[1] + h/2)/h_main,pt,(pt[0] + w, pt[1] + h)])
        
    return points

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber_no_cvbridge')
        self.subscription = self.create_subscription(
            Image,
            'gripper_cam/image_raw',
            self.listener_callback,
            10)
        self.height_publisher = self.create_publisher(Float32, 'height', 10)
        self.current_height_subscriber = self.create_subscription(Float32,'robot_height',self.current_height_setter_callback,10)

        template_paths = [f'mushroom{i}.png' for i in range(1,13)]
        self.templates=[cv2.imread(template_path) for template_path in template_paths] 
        self.position={'x':0,'y':0}

        self.subscription_angle1 = self.create_subscription(Int32,'current_angle1',self.set_theta1_callback, 10)
        self.subscription_angle2 = self.create_subscription(Int32,'current_angle2',self.set_theta2_callback, 10)
        self.subscription_angle3 = self.create_subscription(Int32,'current_angle3',self.set_theta3_callback, 10)

        self.is_auto_set = self.create_subscription(Int32,'auto',self.set_auto, 10)

        self.publisher_angle1 = self.create_publisher(Int32,'angle_theta1', 10)
        self.publisher_angle2 = self.create_publisher(Int32,'angle_theta2', 10)
        self.publisher_angle3 = self.create_publisher(Int32,'angle_theta3', 10)

        self.set_finger_angle_publish =self.create_publisher(Int32,'set_finger_angle',10)

        self.close_gripper_client = self.create_client(Trigger, 'close_gripper')
        self.open_gripper_client = self.create_client(Trigger, 'open_gripper')

        self.height_publisher = self.create_publisher(Float32, 'height', 10)
        
        self.current_point = self.create_subscription(Point, 'point_topic', self.current_point_setter_callback, 10)
        self.target_position_publisher = self.create_publisher(Point, 'target_position', 10)

        self.drop_poistion=np.array([0.66,-0.12])

        self.current_x = 0.0
        self.current_y = 0.0


        self.height=0
        self.theta3=0
        self.theta1=0
        self.theta2=0
        self.auto=False
        self.step=0
        self.time_track=time.time()

        self.link_lengths = [0.325, 0.275, 0.26]
        self.alpha = 0.5
        self.joint_limits= [(-np.pi/2, np.pi/2), (-np.pi/1.5, np.pi/1.5), (-np.pi, np.pi)]
        self.tolerance = 1e-4
        self.max_iter = 100
        self.done=False
        self.move_up=False
        self.move_to_drop_off=False
        self.point_to_go_back_to=None
        self.move_to_pick_up_poistion=False
        self.reset=False

    def current_point_setter_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y

    def current_height_setter_callback(self,msg):
        self.height=msg.data

    def set_auto(self,msg):
        if msg.data>0:
            self.auto=True
            self.set_finger_angle_publish.publish(Int32(data=10))
            self.done=False
            self.move_up=False
            self.move_to_drop_off=False
            self.move_to_pick_up_poistion=False
            self.reset=False

        else:
            self.auto=False
            self.done=False
            self.move_up=False
            self.move_to_drop_off=False
            self.move_to_pick_up_poistion=False
            self.reset=False

    def set_theta3_callback(self,msg):
        self.theta3=msg.data*3.1415/180
    def set_theta1_callback(self,msg):
        self.theta1=msg.data*3.1415/180
    def set_theta2_callback(self,msg):
        self.theta2=msg.data*3.1415/180

    def forward_kinematics(self, theta):
        l1, l2, l3 = self.link_lengths
        t1, t2, t3 = theta

        x = l1 * np.cos(t1) + l2 * np.cos(t1 + t2) + l3 * np.cos(t1 + t2 + t3)
        y = l1 * np.sin(t1) + l2 * np.sin(t1 + t2) + l3 * np.sin(t1 + t2 + t3)
        return np.array([x, y])

    def clamp_angles(self, theta):
        if self.joint_limits is None:
            return theta
        return np.clip(theta, [lim[0] for lim in self.joint_limits], [lim[1] for lim in self.joint_limits])

    def move_to(self, target, initial_theta):
        theta = np.array(initial_theta, dtype=np.float64)
        error = np.array([-target[0],target[1]])
        error_norm = np.linalg.norm(error)
        J = jacobian(self.forward_kinematics, theta).df
        dtheta = self.alpha * pinv(J) @ error
        return dtheta

    def track(self,pt):
        if (time.time()-self.time_track)>0.5 and self.auto:          
            dtheta=self.move_to([pt[0]-0.5,pt[1]-0.5],[self.theta1,self.theta2,self.theta3])
            theta1=self.theta1-dtheta[0]
            theta2=self.theta2-dtheta[1]
            theta3=self.theta3-dtheta[2]
            self.time_track=time.time()
            #print(pt[0]-0.5,pt[1]-0.5,int(math.ceil(self.theta1*180/3.1415)),int(math.ceil(self.theta2*180/3.1415)),int(math.ceil(self.theta3*180/3.1415)),int(math.ceil(theta1*180/3.1415)),int(math.ceil(theta2*180/3.1415)),int(math.ceil(theta3*180/3.1415)))
            
            self.publisher_angle1.publish(Int32(data=int(math.ceil(theta1*180/3.1415))))
            self.publisher_angle2.publish(Int32(data=int(math.ceil(theta2*180/3.1415))))
            self.publisher_angle3.publish(Int32(data=int(math.ceil(theta3*180/3.1415))))
            if ((pt[0]-0.5)**2+(pt[1]-0.5)**2)<0.01 and self.height>-0.015:
                self.height-=0.01
                self.height_publisher.publish(Float32(data=self.height))
            
            if ((pt[0]-0.5)**2+(pt[1]-0.5)**2)<0.01 and self.height<=-0.015:
                self.close_gripper_client.call_async(Trigger.Request())
                self.move_up=True
                self.done=True
                self.close_time=time.time()
        
        

        #     self.time_track=time.time()
        #     if pt[1]<0.4:
        #         self.theta3+=2
        #     elif pt[1]>0.6:
        #         self.theta3-=2
        #     if pt[0]<0.4:
        #         self.theta2+=2
        #     elif pt[0]>0.6:
        #        self.theta2-=2

        #     self.publisher_angle1.publish(Int32(data=self.theta1))
        #     self.publisher_angle2.publish(Int32(data=self.theta2))
        #     self.publisher_angle3.publish(Int32(data=self.theta3))
        #     self.time_track=time.time()
        #     print(pt[0],pt[1],self.theta1,self.theta2,self.theta3)
        # else:
        #     if not self.auto:
        #         print("auto is off")


    def listener_callback(self, msg: Image):
        # Extract image properties from msg
        height = msg.height
        width = msg.width
        encoding = msg.encoding
        data = msg.data

        
        if encoding == 'rgb8':
            img = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
            # Convert RGB to BGR for OpenCV display
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif encoding == 'bgr8':
            img = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
        elif encoding == 'mono8':
            img = np.frombuffer(data, dtype=np.uint8).reshape((height, width))
        else:
            self.get_logger().warn(f'Unsupported encoding: {encoding}')
            return
        cv2.imshow('Image from mushrrom', img)
        output_img=img.copy()
        pt = find_and_draw_templates(img,self.templates )
        print(len(pt))
        if self.move_up:
            print("ready to move up")
            if (time.time()-self.close_time)>3:
                self.height_publisher.publish(Float32(data=0.05))
                #self.done=False
                self.move_up=False
                self.move_to_drop_off=True
                self.point_to_go_back_to=np.array([self.current_x,self.current_y])
                print(f" moving up,point_to_come_back {self.point_to_go_back_to}")
                point = Point()
                point.x =self.drop_poistion[0]
                point.y =self.drop_poistion[1]
                self.target_position_publisher.publish(point)
                self.close_time=time.time()

        elif self.move_to_drop_off:
            if (time.time()-self.close_time)<2:
                print("waiting drop off")
            elif np.linalg.norm([self.current_x,self.current_y]-self.drop_poistion)<0.1:
                self.open_gripper_client.call_async(Trigger.Request())
                self.move_to_pick_up_poistion=True
                point = Point()
                point.x =self.point_to_go_back_to[0]
                point.y =self.point_to_go_back_to[1]
                self.target_position_publisher.publish(point)
                self.move_to_drop_off=False
                print("drop off done")
                self.close_time=time.time()

        elif self.move_to_pick_up_poistion:
            if (time.time()-self.close_time)<2:
                 print("waiting moving back")
            elif np.linalg.norm([self.current_x,self.current_y]-self.point_to_go_back_to)<0.1:
                self.reset=True
                self.move_to_pick_up_poistion=False
                print("back done")
                self.close_time=time.time()
        
        elif self.reset:
            if (time.time()-self.close_time)<2:
                print("waiting to reset")
            else:
                self.done=False
                self.reset=False

        elif pt:
            cost=np.array([(p[0]-0.5)**2+(p[1]-0.5)**2 for p in pt])
            min_val=np.argmin(cost)
            if not self.done:
                self.track(pt[min_val])
            cv2.rectangle(output_img, pt[min_val][2],  pt[min_val][3], (0, 255, 0), 2)


                

        cv2.imshow('Image from mushrrom', output_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
