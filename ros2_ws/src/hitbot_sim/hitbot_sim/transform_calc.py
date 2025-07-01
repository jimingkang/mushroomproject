import numpy as np


class RobotTransform:
    def __init__(self,link_length):
        self.a1=link_length[0]
        self.a2=link_length[1]
        self.a3=link_length[2]

    def dh_transform(self, a, alpha, d, theta):
        """
        Compute the Denavit-Hartenberg transformation matrix.
        
        Parameters:
        a     -- link length
        alpha -- link twist
        d     -- link offset
        theta -- joint angle
        
        Returns:
        4x4 numpy array representing the transformation matrix
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,       sa,      ca,      d],
            [0,        0,       0,      1]
        ])

    def tranform_0_3(self,current_angles):
        T0_1 = self.dh_transform(self.a1, 0, 0, current_angles[0])
        T1_2 = self.dh_transform(self.a2, 0, 0, current_angles[1])
        T2_3 = self.dh_transform(self.a3, 0, 0, current_angles[2])
        T0_3 = T0_1 @ T1_2 @ T2_3
        #T3_0 = np.linalg.inv(T0_3)
        return T0_3
    
    def tranform_point3_0(self,point=[0,0],current_angles=[0,0,0]):
        P3 = np.array([point[0], point[1], 0, 1]) 
        T0_3 = self.tranform_0_3(current_angles)
        P0 = T0_3 @ P3
        return (P0)
# a1 = 1.0  
# a2 = 0.8  
# a3 = 0.2

# rt=robot_transfrom([a1,a2,a3])

# print(rt.tranform_point3_0([0,0],[0,np.pi/2,np.pi/2]))