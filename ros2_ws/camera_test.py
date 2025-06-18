import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

from geometry_msgs.msg import Point
from std_msgs.msg import Int32, Float32
import time
#from scipy.differentiate import jacobian
from scipy.linalg import pinv
import math
from std_srvs.srv import Trigger
import numpy
from sensor_msgs.msg import JointState

from scipy.optimize._numdiff import approx_derivative 
def jacobian(func, x):
    return approx_derivative(func, x, method='forward')

import glob


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
        self.cap = cv2.VideoCapture(0)
        #self.height_publisher = self.create_publisher(Float32, 'height', 10)
        #self.current_height_subscriber = self.create_subscription(Float32,'robot_height',self.current_height_setter_callback,10)

        template_paths = glob.glob("temp_work_dir_raman/mushroomimg/*.png")
        self.templates=[cv2.imread(template_path) for template_path in template_paths] 
        self.position={'x':0,'y':0}
        self.joint_state_sub = self.create_subscription(JointState, "/hitbot/joint_states",self.set_current_angles, 10)
        self.height=0
        self.theta3=0
        self.theta1=0
        self.theta2=0
        self.alpha = 0.5
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.done=False
        self.reset=False

    def set_current_angles(self,msg):
        self.height = msg.position[0]
        self.theta1 = msg.position[1]
        self.theta1 = msg.position[2]
        self.theta1 = msg.position[3]
    
    def forward_kinematics(self, theta):
        l1, l2, l3 = self.link_lengths
        t1, t2, t3 = theta

        x = l1 * np.cos(t1) + l2 * np.cos(t1 + t2) + l3 * np.cos(t1 + t2 + t3)
        y = l1 * np.sin(t1) + l2 * np.sin(t1 + t2) + l3 * np.sin(t1 + t2 + t3)
        return np.array([x, y])
    
    def move_to(self, target, initial_theta):
        theta = np.array(initial_theta, dtype=np.float64)
        error = np.array([-target[0],target[1]])
        error_norm = np.linalg.norm(error)
        J = jacobian(self.forward_kinematics, theta).df
        dtheta = self.alpha * pinv(J) @ error
        

        return dtheta
    
    def track(self,pt):
        dtheta=self.move_to([pt[0]-0.5,pt[1]-0.5],[self.theta1,self.theta2,self.theta3])
        theta1=self.theta1-dtheta[0]
        theta2=self.theta2-dtheta[1]
        theta3=self.theta3-dtheta[2]
        self.time_track=time.time()
    def timer_callback(self):
        ret, frame = self.cap.read()
        # Display the resulting frame
        cv2.imshow('Camera Test', frame)
        output_img=frame.copy()
        pt = find_and_draw_templates(frame,self.templates )
        if pt:
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


print("✅ Camera opened successfully. Press 'q' to quit.")


# template_paths=glob.glob("temp_work_dir_raman/mushroomimg/*.png")
# templates=[cv2.imread(template_path) for template_path in template_paths] 
# while True:

#     # Capture frame-by-frame
#     ret, frame = cap.read()

#     # If frame read is not successful, break
#     if not ret:
#         print("❌ Can't receive frame (stream end?). Exiting ...")
#         break

#     # Display the resulting frame
#     cv2.imshow('Camera Test', frame)
#     output_img=frame.copy()
#     pt = find_and_draw_templates(frame,templates )
#     if pt:
#         cost=np.array([(p[0]-0.5)**2+(p[1]-0.5)**2 for p in pt])
#         min_val=np.argmin(cost)
#         cv2.rectangle(output_img, pt[min_val][2],  pt[min_val][3], (0, 255, 0), 2)


            

#     cv2.imshow('Image from mushrrom', output_img)
#     cv2.waitKey(1)
#     # Break the loop on 'q' key press
#     if cv2.waitKey(1) == ord('q'):
#         break

# # Release the capture and destroy the window
# cap.release()
# cv2.destroyAllWindows()
