import math
from ikpy.chain import Chain
import numpy as np
import matplotlib.pyplot as plt
import trimesh
import octomap
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# Load the OctoMap from a .bt file

from scipy.optimize import root

class OldSolver:
    def __init__(self,link_lengths=np.array([0.325, 0.275, 0.30])):
        self.link_lengths = link_lengths

    def forward_kinematics_from_angles(self,theta):
        theta1, theta2, theta3 = theta
        x = self.link_lengths[0] * np.cos(theta1) + self.link_lengths[1] * np.cos(theta1 + theta2) + self.link_lengths[2] * np.cos(theta1 + theta2 + theta3)
        y = self.link_lengths[0] * np.sin(theta1) + self.link_lengths[1] * np.sin(theta1 + theta2) + self.link_lengths[2] * np.sin(theta1 + theta2 + theta3)
        return np.array([x, y])

    def forward_kinematics_from_angles_all(self,theta):
        theta1, theta2, theta3 = theta
        x1 = self.link_lengths[0] * np.cos(theta1)
        y1 = self.link_lengths[0] * np.sin(theta1)
        x2 = x1 + self.link_lengths[1] * np.cos(theta1 + theta2)
        y2 = y1 + self.link_lengths[1] * np.sin(theta1 + theta2)
        x3 = x2 + self.link_lengths[2] * np.cos(theta1 + theta2 + theta3)
        y3 = y2 + self.link_lengths[2] * np.sin(theta1 + theta2 + theta3)
        return np.array([[x1, y1], [x2, y2], [x3, y3]])

    def error_function(self,theta):
        x,y=self.target-self.forward_kinematics_from_angles(theta)
        dx=x*0.001
        dy=y*0.001
        return np.array([self.compute_dtheta1(dx, dy, theta[0], theta[1], theta[2]),self.compute_dtheta2(dx, dy, theta[0], theta[1], theta[2]),self.compute_dtheta3(dx, dy, theta[0], theta[1], theta[2])])

    def compute_dtheta1(self,dx, dy, theta1, theta2, theta3):
        r1,r2,r3=self.link_lengths
        jx = -r1 * np.sin(theta1) - r2 * np.sin(theta1 + theta2) - r3 * np.sin(theta1 + theta2 + theta3)
        jy =  r1 * np.cos(theta1) + r2 * np.cos(theta1 + theta2) + r3 * np.cos(theta1 + theta2 + theta3)
        J_theta1 = np.array([jx, jy])
        delta_p = np.array([dx, dy])
        dtheta1 = np.dot(J_theta1, delta_p) / np.dot(J_theta1, J_theta1)
        return dtheta1

    def compute_dtheta2(self,dx, dy, theta1, theta2, theta3):
        r1,r2,r3=self.link_lengths
        jx = - r2 * np.sin(theta1 + theta2) - r3 * np.sin(theta1 + theta2 + theta3)
        jy = r2 * np.cos(theta1 + theta2) + r3 * np.cos(theta1 + theta2 + theta3)
        J_theta1 = np.array([jx, jy])
        delta_p = np.array([dx, dy])
        dtheta1 = np.dot(J_theta1, delta_p) / np.dot(J_theta1, J_theta1)
        return dtheta1

    def compute_dtheta3(self,dx, dy, theta1, theta2, theta3):
        r1,r2,r3=self.link_lengths
        jx = - r3 * np.sin(theta1 + theta2 + theta3)
        jy = r3 * np.cos(theta1 + theta2 + theta3)
        J_theta1 = np.array([jx, jy])
        delta_p = np.array([dx, dy])
        dtheta1 = np.dot(J_theta1, delta_p) / np.dot(J_theta1, J_theta1)
        return dtheta1

    def cost(self, ik_success, current_angle):
        ik_success = np.array(ik_success)
        current_value = self.forward_kinematics_from_angles_all(current_angle)
        angle_costs=[]
        # weights = np.array([[2.0,2.0],[ 1.0,1.0], [1,1]])
        # weights2 = np.array([[1.5,1.5],[ 1.0,1.0], [1,1]])

        for i in ik_success:
            #values=self.forward_kinematics_from_angles_all(i)
            val1=abs(i[0]-current_angle[0])*100 if i[0]>0 else abs(i[0]-current_angle[0])*80
            val2=abs(i[1]-current_angle[1]) *1000
            val3=abs(i[2]-current_angle[2])*8 if i[2]>0  else abs(i[2]-current_angle[2])*10
            angle_costs.append(np.linalg.norm(val1+val2+val3))

        return np.array(angle_costs)

    def solve(self,target,current_angle):
        self.target= np.array(target)
        limits=[[-np.pi/2,np.pi/2],[-np.pi/1.3,np.pi/1.3],[-np.pi/2,np.pi/2]]
        initial_guesses=[[np.random.uniform(*limits[i]) for i in range(3)] for i in range(0,100)]
        ik_success=[]
        for guess in initial_guesses:
            sol=root(self.error_function, guess, method='hybr')
            if sol.success:
                error = np.linalg.norm(self.target-self.forward_kinematics_from_angles(sol.x))
                x = sol.x

                within_limits = all(
                    limits[i][0] <= x[i] <= limits[i][1]
                    for i in range(3)
                )

                if error < 0.01 and within_limits:
                    ik_success.append(x)
        if len(ik_success):
            ik_success=np.array(ik_success)
            angle_cost=self.cost(ik_success, current_angle)
            return ik_success,angle_cost
        else:
            return target,current_angle

    def get_angle_to_target(self, target, current_angle):
        """
        Computes the optimal joint configuration (angle) required to reach a given target from the current angle.

        This function uses the `solve` method to calculate possible solutions and their associated angle costs,
        then selects and returns the configuration with the minimum cost.

        Parameters:
        ----------
        target : array-like
            The desired position or state that the system should reach.
        current_angle : array-like or float
            The current angle or joint configuration of the system.

        Returns:
        -------
        array-like or float
            The angle configuration from the list of solutions that has the minimum associated cost.
        """
        solved, angle_cost = self.solve(target, current_angle)
        min_config = np.argmin(angle_cost)
        try:
            return solved[min_config]
        except IndexError:
            print(f"Error: No solution at index {min_config} (total solutions: {len(solved)})")
            return None  # or raise a more descriptive exception

        #return solved[min_config]
    def normalize_angle(self,deg):
        deg = ((deg + 180) % 360) - 180
        return round(deg, 1)

    def get_robot_angle_in_degree(self, target, current_angle=[0,0,0]):
        ans=self.get_angle_to_target(target, current_angle)
        if ans is None:  # Check if IK failed
            print("ERROR: No valid joint angles (IK failed)!")
            return [0.0, 0.0, 0.0]  # Default safe value (or raise an exception)
        theta1 = ans[0] * 180.0 / math.pi
        theta2 = ans[1] * 180.0 / math.pi
        theta3 = ans[2] * 180.0 / math.pi

    # 累积角度
        angles = [theta1, theta1 + theta2,theta1 + theta2 + theta3]
        normalized = [self.normalize_angle(a) for a in angles]
        return normalized
class Solver:
    def __init__(self, link_lengths=np.array([0.325, 0.275, 0.3])):
        self.link_lengths = link_lengths

    def forward_kinematics_from_angles(self, theta):
        t1, t2, t3 = theta
        x = (self.link_lengths[0] * np.cos(t1) +
             self.link_lengths[1] * np.cos(t1 + t2) +
             self.link_lengths[2] * np.cos(t1 + t2 + t3))
        y = (self.link_lengths[0] * np.sin(t1) +
             self.link_lengths[1] * np.sin(t1 + t2) +
             self.link_lengths[2] * np.sin(t1 + t2 + t3))
        return np.array([x, y])

    def jacobian(self, theta):
        r1, r2, r3 = self.link_lengths
        t1, t2, t3 = theta
        j11 = -r1*np.sin(t1) - r2*np.sin(t1+t2) - r3*np.sin(t1+t2+t3)
        j12 = -r2*np.sin(t1+t2) - r3*np.sin(t1+t2+t3)
        j13 = -r3*np.sin(t1+t2+t3)
        j21 = r1*np.cos(t1) + r2*np.cos(t1+t2) + r3*np.cos(t1+t2+t3)
        j22 = r2*np.cos(t1+t2) + r3*np.cos(t1+t2+t3)
        j23 = r3*np.cos(t1+t2+t3)
        return np.array([[j11, j12, j13],
                         [j21, j22, j23]])

    def inverse_kinematics(self, target, initial_guess, max_iter=100, tol=1e-5):
        theta = np.array(initial_guess, dtype=float)
        for _ in range(max_iter):
            current_pos = self.forward_kinematics_from_angles(theta)
            error = target - current_pos
            if np.linalg.norm(error) < tol:
                break
            J = self.jacobian(theta)
            dtheta = np.linalg.pinv(J) @ error
            theta += dtheta

        theta = (theta + np.pi) % (2 * np.pi) - np.pi

        # ✅ 转换为绝对角度
        theta_abs = np.cumsum(theta)

        # ✅ 再次归一化到 [-π, π]（防止累积越界）
        theta_abs = (theta_abs + np.pi) % (2 * np.pi) - np.pi
        return theta_abs

def load_octomap(file_path):
    occupancy_map = octomap.OcTree(0.05)  #self.process_octomap(msg)# 5 cm resolution
    if isinstance(file_path, bytes):  
        file_path = file_path.decode("utf-8")  # ? Convert bytes to string

        # ? Load the Octomap from the file
    if occupancy_map.readBinary(file_path.encode()):  
        print("? Octomap loaded successfully!")
        print(f"Number of occupied nodes: {sum(1 for _ in occupancy_map.begin_leafs())}")
    else:
        print("? Error: Could not load Octomap from file!")
    return occupancy_map
def get_occupied_voxels(octree):
    occupied_voxels = set()  # Use a set for faster lookups
    for node in octree.begin_leafs():
        if octree.isNodeOccupied(node):
            x, y, z = node.getCoordinate()
            occupied_voxels.add((x, z))  # Store XZ coordinates
    return occupied_voxels

def check_joint_collision(p1, occupied_voxels, voxel_size=0.1):
    # Bresenham's line algorithm to check for intersections
    #x1, z1 = p1

    nearest_node, distance = find_nearest_node(p1, occupied_voxels)
    print(distance)
    if distance<0.1:
        return True
    else: 
        return False
def find_nearest_node(p1, occupied_voxels):
    min_distance = float('inf')  # Initialize with a large value
    nearest_node = None

    for (x2, z2) in occupied_voxels:
        distance = math.sqrt((x2 - p1[0]) ** 2 + (z2 - p1[2]) ** 2)  # Euclidean distance
        if distance < min_distance:
            min_distance = distance
            nearest_node = (x2, z2)

    return nearest_node, min_distance
# Check if the robot's links collide with the obstacles in the XZ plane
def check_robot_collision_xz(chain, joint_positions, occupied_voxels):
    # Get the transformation matrices for each joint
    #frames = chain.forward_kinematics(joint_positions, full_kinematics=True)
    # Extract the XZ positions of the joints
    #joint_positions_xz = [frame[[0, 2], 3] for frame in frames]  # Extract X and Z coordinates
    # Extract the XY positions of the joints
    #joint_positions_xy = [frame[:2, 3] for frame in frames]  # Extract X and Y coordinates

    joint_positions_xz =joint_positions#np.array(joint_positions)[:,:2].tolist() # Extract X and Y coordinates
    # Check for collisions between each pair of consecutive joints
    for i in range(len(joint_positions_xz) - 1):
        p1 = joint_positions_xz[i]
        p2 = joint_positions_xz[i + 1]
        if check_joint_collision(p1, occupied_voxels):
            print(f"Collision detected between joint {i} and joint {i + 1}!")
            return True
        #if check_line_collision(p1, p2, occupied_voxels):
        #    print(f"Collision detected between joint {i} and joint {i + 1}!")
        #    return True
    print("No collision detected.")
    return False

# Example usage
if __name__ == "__main__":

    # Define the transformation matrix (example values)
    # Rotation matrix R (3x3)
    R = np.array([
        [0, -1, 0], #[1,0,0]
        [0, 0, -1],  # [0, 1, 0],
        [1, 0, 0]    #        [0,0,1]
    ])



    # Translation vector t (3x1) 
    #t = np.array([0.0, 0.0, 0.0])
    # Construct the 4x4 homogeneous transformation matrix
    #T = np.eye(4)  # Start with an identity matrix
    #T[:3, :3] = R  # Set the rotation part
    #T[:3, 3] = t   # Set the translation part

    # Point in hitbot_arm  coordinates (homogeneous coordinates)
    target_position = np.array([0.8, 0.0, 0.0, 1])  # [x, y, z, 1]

    # Transform to camera coordinates
    #target_camera_position = T @ target_position  # Matrix multiplication

    # Extract the 3D coordinates (ignore the homogeneous coordinate)
    #P_camera_3d = target_camera_position[:3]

 
    # Load the OctoMap
    octree = load_octomap("/home/jimmy/Downloads/newmap.bt")
    urdf_file = "/home/jimmy/Downloads/mushroomproject/ros2_ws/src/hitbot_sim/hitbot_sim/scara_ik.xml"
    scara_arm = Chain.from_urdf_file(urdf_file,base_elements=["base_link"] )
    
    # Define joint positions (example)
    target_position = [0.5, 0.4, 0.0]  # Replace with your target position
    target_orientation = [0, 0, 0.0]  # Replace with your target orientation (roll, pitch, yaw)
    
 

    joint_angles = scara_arm.inverse_kinematics(target_position,target_orientation )

    joint_positions_deg = [pos * 180 / math.pi for pos in joint_angles]
    print("Computed Joint Angles:", joint_positions_deg)

    # Compute forward kinematics to get the positions of all joints
    joint_positions = scara_arm.forward_kinematics(joint_angles, full_kinematics=True)
    print(joint_positions[-1][:3, 3])
    # Extract the positions of all joints
    joint_positions = [frame[:3, 3] for frame in joint_positions]

    camera_joint_positions=R@np.transpose(np.matrix(joint_positions))
    camera_joint_positions=np.asarray(np.transpose(camera_joint_positions))
    print("Computed Joint joint_positions:",camera_joint_positions)
   
    
    occupied_voxels = get_occupied_voxels(octree)
    # Check for collisions
    #check_robot_collision_xz(scara_arm, camera_joint_positions, occupied_voxels)

    solver = Solver()
    target = np.array([0.5, -0.4])
    theta_init = [0, 0, 0]
    result = solver.inverse_kinematics(target, theta_init)
    print("IK解:", np.degrees(result))
    print("验证FK:", solver.forward_kinematics_from_angles(result))
    oldsolver = OldSolver()
    RES=oldsolver.get_robot_angle_in_degree(target)
    print("RES IK解:", result)
