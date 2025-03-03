

import math
import numpy as np
import trimesh
import octomap

# Project the OctoMap into the XZ plane
def project_octomap_to_xy(octree, y_height=0.0, tolerance=0.1):
    occupied_voxels = []
    for node in octree.begin_leafs():
        if octree.isNodeOccupied(node):
            x, y, z = node.getCoordinate()
            if abs(y - y_height) < tolerance:  # Consider only voxels near the specified Y height
                occupied_voxels.append((x, z))
    return occupied_voxels


def get_occupied_voxels(octree):
    occupied_voxels = set()  # Use a set for faster lookups
    for node in octree.begin_leafs():
        if octree.isNodeOccupied(node):
            x, y, z = node.getCoordinate()
            occupied_voxels.add((x, z))  # Store XZ coordinates
    return occupied_voxels

def find_nearest_node(x1, z1, occupied_voxels):
    min_distance = float('inf')  # Initialize with a large value
    nearest_node = None

    for (x2, z2) in occupied_voxels:
        distance = math.sqrt((x2 - x1) ** 2 + (z2 - z1) ** 2)  # Euclidean distance
        if distance < min_distance:
            min_distance = distance
            nearest_node = (x2, z2)

    return nearest_node, min_distance

def check_joint_collision(p1, occupied_voxels, voxel_size=0.1):
    # Bresenham's line algorithm to check for intersections
    x1, z1 = p1

    nearest_node, distance = find_nearest_node(x1, z1, occupied_voxels)
    print(distance)
    if distance<0.1:
        return True
    else: 
        return False



# Check if a line segment intersects with any occupied voxel
def check_line_collision(p1, p2, occupied_voxels, voxel_size=0.1):
    # Bresenham's line algorithm to check for intersections
    x1, z1 = p1
    x2, z2 = p2
    dx = abs(x2 - x1)
    dz = abs(z2 - z1)
    sx = 1 if x1 < x2 else -1
    sz = 1 if z1 < z2 else -1
    err = dx - dz
    i=0
    nearest_node, distance = find_nearest_node(x1, z1, occupied_voxels)
    print(distance)
    if distance<0.1:
        return True
    else: 
        return False
    while False:
        # Check if the current point is in the occupied voxels
        nearest_node, distance = find_nearest_node(x1, z1, occupied_voxels)
        if distance<0.1:
            return True
        if x1 == x2 and z1 == z2:
            break
        e2 = 2 * err
        if e2 > -dz:
            err -= dz
            x1 += sx
        if e2 < dx:
            err += dx
            z1 += sz
            i=i+1
    return False

# Check if the robot's links collide with the obstacles in the XZ plane
def check_robot_collision_xy(chain, joint_positions, occupied_voxels):
    # Get the transformation matrices for each joint
    frames = chain.forward_kinematics(joint_positions, full_kinematics=True)
    
    # Extract the XZ positions of the joints
    #joint_positions_xz = [frame[[0, 2], 3] for frame in frames]  # Extract X and Z coordinates
    

    # Extract the XY positions of the joints
    joint_positions_xy = [frame[:2, 3] for frame in frames]  # Extract X and Y coordinates

    # Check for collisions between each pair of consecutive joints
    for i in range(len(joint_positions_xy) - 1):
        p1 = joint_positions_xy[i]
        p2 = joint_positions_xy[i + 1]
        if check_joint_collision(p1, occupied_voxels):
            print(f"Collision detected between joint {i} and joint {i + 1}!")
            return True
        #if check_line_collision(p1, p2, occupied_voxels):
        #    print(f"Collision detected between joint {i} and joint {i + 1}!")
        #    return True
    print("No collision detected.")
    return False


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

# Check if a point is occupied in the OctoMap
def is_point_occupied(octree, point):
    try:
        node = octree.search(point)
        if node is None:
            print("Point is in an unknown region or outside the map.")
            return False
        else:
            print("Point is occupied:", octree.isNodeOccupied(node))
            return True
    except octomap.NullPointerException as e:
        print("NullPointerException:", e)
        return True

# Check if a mesh collides with the OctoMap
def check_mesh_collision(octree, mesh, transform):
    # Transform the mesh vertices to the world frame
    vertices = mesh.vertices
    transformed_vertices = trimesh.transform_points(vertices, transform)
    
    # Check if any vertex is inside an occupied voxel
    for vertex in transformed_vertices:
        if is_point_occupied(octree, vertex):
            return True
    return False

# Main function to check robot collision with OctoMap
def check_robot_collision(chain, joint_positions, octree):
    # Get the transformation matrices for each joint
    frames = chain.forward_kinematics(joint_positions, full_kinematics=True)
    
    # Load collision meshes for each link
    collision_meshes = [
        trimesh.load_mesh(f"/home/jimmy/Downloads/mushroomproject/ros2_ws/src/hitbot_sim/meshes/Z-Arm_10042C0/base_link.STL"),
        trimesh.load_mesh(f"/home/jimmy/Downloads/mushroomproject/ros2_ws/src/hitbot_sim/meshes/Z-Arm_10042C0/Link1.STL"),
        trimesh.load_mesh(f"/home/jimmy/Downloads/mushroomproject/ros2_ws/src/hitbot_sim/meshes/Z-Arm_10042C0/Link2.STL"),
        trimesh.load_mesh(f"/home/jimmy/Downloads/mushroomproject/ros2_ws/src/hitbot_sim/meshes/Z-Arm_10042C0/Link3.STL"),
        trimesh.load_mesh(f"/home/jimmy/Downloads/mushroomproject/ros2_ws/src/hitbot_sim/meshes/Z-Arm_10042C0/Link4.STL"),
        trimesh.load_mesh(f"/home/jimmy/Downloads/mushroomproject/ros2_ws/src/hitbot_sim/meshes/Z-Arm_10042C0/Link5.STL"),
    ]
    
    # Check for collisions between each link and the OctoMap
    for i, (mesh, frame) in enumerate(zip(collision_meshes, frames)):
        #if i<2:
        #    continue
        if check_mesh_collision(octree, mesh, frame):
            print(f"Collision detected for link {i}!")
            return True
    print("No collision detected.")
    return False

def transform_point(point, transformation_matrix):
    # Convert the point to homogeneous coordinates
    point_homogeneous = np.append(point, 1)
    
    # Apply the transformation
    transformed_point_homogeneous = np.dot(transformation_matrix, point_homogeneous)
    
    # Convert back to Cartesian coordinates
    transformed_point = transformed_point_homogeneous[:3]
    return transformed_point
# Example usage
if __name__ == "__main__":

    R = np.array([[0, 0, 1],  # Rotation matrix
              [0, -1, 0],
              [1, 0, 0]])
    t = np.array([0, 0.0, 0.0])  # Translation vector

    # Construct the 4x4 transformation matrix
    T_A_to_O = np.eye(4)
    T_A_to_O[:3, :3] = R
    T_A_to_O[:3, 3] = t

    
    # Load the OctoMap
    octree = load_octomap("/home/jimmy/Downloads/newmap.bt")
    
    # Load the robot's URDF and create the chain
    from ikpy.chain import Chain
    urdf_file = "/home/jimmy/Downloads/mushroomproject/ros2_ws/src/hitbot_sim/urdf/Z-Arm_10042C0_gazebo_ik.urdf"
    chain = Chain.from_urdf_file(urdf_file)
    
    # Define joint positions (example)
    target_position = [0.5, 0.0, 0.0]  # Replace with your target position
    target_orientation = [0, 0, 0.0]  # Replace with your target orientation (roll, pitch, yaw)
    
    # Compute inverse kinematics
    joint_angles = chain.inverse_kinematics(target_position, target_orientation)
    print(joint_angles)


    # Compute forward kinematics to get the Cartesian positions of all joints
    joint_positions = chain.forward_kinematics(joint_angles, full_kinematics=True)

    
    occupied_voxels = get_occupied_voxels(octree)
    # Check for collisions
    check_robot_collision_xy(chain, joint_positions, occupied_voxels)