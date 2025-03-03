import math
from ikpy.chain import Chain
import numpy as np
import matplotlib.pyplot as plt
import trimesh
import octomap
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# Load the OctoMap from a .bt file
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
    scara_arm = Chain.from_urdf_file(urdf_file)
    
    # Define joint positions (example)
    target_position = [0.8, 0.1, 0.0]  # Replace with your target position
    target_orientation = [0, 0, 0.0]  # Replace with your target orientation (roll, pitch, yaw)
    
 

    joint_angles = scara_arm.inverse_kinematics(target_position,target_orientation)
    print("Computed Joint Angles:", joint_angles)

    # Compute forward kinematics to get the positions of all joints
    joint_positions = scara_arm.forward_kinematics(joint_angles, full_kinematics=True)

    # Extract the positions of all joints
    joint_positions = [frame[:3, 3] for frame in joint_positions]

    camera_joint_positions=R@np.transpose(np.matrix(joint_positions))
    camera_joint_positions=np.asarray(np.transpose(camera_joint_positions))
    print("Computed Joint joint_positions:",camera_joint_positions)
   
    
    occupied_voxels = get_occupied_voxels(octree)
    # Check for collisions
    check_robot_collision_xz(scara_arm, camera_joint_positions, occupied_voxels)

