import io
import subprocess
import sys
import os
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int64,String
from sensor_msgs.msg import JointState
from hitbot_msgs.srv import *
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import DisplayTrajectory
import redis
from octomap_msgs.msg import Octomap
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import numpy as np
import random
import octomap
from scipy.spatial import KDTree
import fcl


redis_server='172.27.34.62'
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)

os.chdir(os.path.expanduser('~'))
#sys.path.append("./ws_moveit/src/hitbot")  ## get import pass: hitbot_interface.py
#from .hitbot_interface import HitbotInterface
from .HitbotInterface import HitbotInterface

class HitbotController(Node):
    def __init__(self):
        super().__init__('hitbot_controller')

        self.SetGPIO_srv = self.create_service(SetGPIO, 'set_gpio', self.set_gpio_callback)
        self.GetGPIOOut_srv = self.create_service(GetGPIOOut, 'get_gpio_out', self.get_gpio_out_callback)
        self.GetGPIOIn_srv = self.create_service(GetGPIOIn, 'get_gpio_in', self.get_gpio_in_callback)
        self.SetDragTeach_srv = self.create_service(SetDragTeach, 'set_drag_teach', self.set_drag_teach_callback)
        self.JointHome_srv = self.create_service(JointHome, 'joint_home', self.joint_home_callback)
        self.NewMovejXYZ_srv = self.create_service(NewMovejXYZ, 'new_movej_xyz_lr', self.new_movej_xyz_lr_callback)
        self.NewMovejAngle_srv = self.create_service(NewMovejAngle, 'new_movej_angle', self.new_movej_angle_callback)


        self.hitbot_x = 0
        self.hitbot_y = 0
        self.hitbot_z = 0
        self.hitbot_r = 0

        self.hitbot_x_publisher = self.create_publisher(String, '/hitbot_x', 10)
        self.hitbot_y_publisher = self.create_publisher(String, '/hitbot_y', 10)
        self.hitbot_z_publisher = self.create_publisher(String, '/hitbot_z', 10)
        self.hitbot_r_publisher = self.create_publisher(String, '/hitbot_r', 10)
        self.camera_xyz_publisher = self.create_publisher(String, '/camera_xyz', 10)
        
        self.xyz_sub = self.create_subscription(String,"/hitbot_end_xyz",self.hitbot_end_xyzr_callback,10)
        
        # Joint State Publisher
        self.joint_state_pub = self.create_publisher(JointState, "/hitbot/joint_states", 10)

        # Timer to publish joint states at 50Hz (20ms)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Define joint names (Modify according to your HitBot model)
        self.joint_names = ["joint1", "joint2", "joint3", "joint4"]
       

        self.joint_command_sub = self.create_subscription(
            DisplayTrajectory,
            "/display_planned_path",
            #"/scara_arm_controller/joint_trajectory",
            self.joint_command_callback,
            10
        )
        # Publisher to send commands to HitBot controller
        self.hitbot_command_pub = self.create_publisher(
            Float64MultiArray,
            "/hitbot/set_joint_positions",
            10
        )


        self.robot_id = 92  ## 123 is robot_id, Modify it to your own
        self.robot = HitbotInterface(self.robot_id)

        self.init_robot()

        self.subscription = self.create_subscription(Octomap,'/rtabmap/octomap_binary',self.octomap_callback,10)

        # Publisher for RRT Path Visualization
        self.path_publisher = self.create_publisher(Marker, 'rrt_path', 10)
        self.fcl_octree = None  # FCL Octree for collision checking
        self.occupancy_map = None  # Store Octomap for collision checking
        self.bounds = (-2.0, 2.0, -2.0, 2.0, -2.0, 2.0)  # Workspace bounds (xmin, xmax, ymin, ymax, zmin, zmax)

    def octomap_callback(self, msg):
        """ Callback to receive and process Octomap data. """


        file_path="/home/a/Downloads/newmap.bt"
        octomap_size = len(msg.data)
        self.get_logger().info(f"octomap_size: {octomap_size},{self.robot.x/1000}{self.robot.y/1000},{self.robot.z/1000}")
        if not msg.data:
            return
        output=subprocess.check_output(["ros2", "run", "octomap_server", "octomap_saver_node", "--ros-args", "-p" ,"octomap_path:=/home/a/Downloads/newmap.bt" ,"-r" ,"octomap_binary:=/rtabmap/octomap_binary" ],text=True)

        

        #with open(file_path, "wb") as f:
        #    title=f"# Octomap OcTree binary file\n# \n".encode("utf-8")
        #    f.write(title)
        #    #octree_tile=f"# OcTree v1.0\n# Dimensions: 256 256 256\n# DataType: uint8\n"..encode("utf-8")
       #     #f.write(octree_tile)  # ? Write header
        #    header = f"id ColorOcTree\nsize {octomap_size}\nres 0.05\ndata\n".encode("utf-8")
        #    f.write(header)  # ? Write header
        #    data = bytearray(msg.data)
        #    f.write(data)
        #    f.flush()  # ? Ensure data is written to disk
        #    f.close()


        self.occupancy_map = octomap.OcTree(0.05)  #self.process_octomap(msg)# 5 cm resolution
  
        #file_path = file_path.decode("utf-8")
        if isinstance(file_path, bytes):  
            file_path = file_path.decode("utf-8")  # ? Convert bytes to string


        # ? Load the Octomap from the file
        if self.occupancy_map.readBinary(file_path.encode()):  
            print("? Octomap loaded successfully!")
            print(f"Number of occupied nodes: {sum(1 for _ in self.occupancy_map.begin_leafs())}")
        else:
            print("? Error: Could not load Octomap from file!")

        self.get_logger().info(f"Received  occupancy_map . {self.occupancy_map is None}")
        if self.occupancy_map:
            start = (-self.robot.y/1000, self.robot.z/1000, self.robot.x/1000)  # Example start position
            goal = (-(self.robot.y+200)/1000, self.robot.z/1000, self.robot.x/1000)   # Example goal position
            #self.convert_to_fcl_objects()#fcl.OcTree(self.occupancy_map)  # ? Convert Octomap to FCL Octree

            self.get_logger().info("? Octomap converted to FCL Octree!")

            self.get_logger().info(f"before rrt planning")
            path = self.rrt_planning(start, goal, max_iter=500)

            if path:

                self.get_logger().info(f"get apath {path}")
                self.visualize_path(path)
            else:
                self.get_logger().warn("No valid path found.")

    def process_octomap(self, msg):
        """ Convert Octomap message to an occupancy grid. """
        try:
            octo_tree = octomap.OcTree(0.05)  # 5 cm resolution
            binary_data = bytearray(msg.data)  # Convert to bytes
            octo_tree.readBinary(binary_data)
            #nod = octo_tree.search((0.4, 0, 0))
            #self.get_logger().info(f"Received process_octomap data. {nod}")
            return octo_tree
        except Exception as e:
            self.get_logger().error(f"Failed to process Octomap: {e}")
            return None
    def convert_to_fcl_objects(self):
        """ Convert Octomap Occupied Nodes into FCL Collision Objects """
        if not self.occupancy_map:
            self.get_logger().error("? No Octomap data available!")
            return

        occupied_voxels= []  # Clear old objects

        #for node in self.occupancy_map.begin_leafs():
            #if self.occupancy_map.isNodeOccupied(node):  # ? Only process occupied nodes
                #occupied_voxels.append([node.getX(),node.getY(),node.getZ()])
                #key = node.getKey()
                #x, y, z = self.occupancy_map.keyToCoord(key)  # ? Convert key to world coordinates
        #        # ? Convert each occupied node to an FCL Box (representing the voxel)
                #size = node.getSize()
                #self.get_logger().info(f"Node geteX{node.getX()}")
                #box = fcl.Box(size, size, size)  # ? Box matching Octomap voxel size
                #box_tf = fcl.Transform([x, y, z])  # ? Box position
                #occupied_voxels.append(fcl.CollisionObject(box, box_tf))


                # ? Define a simple bounding volume for each voxel (Cube)
                #vertices = [
                #    (x - size / 2, y - size / 2, z - size / 2),
                  #  (x + size / 2, y - size / 2, z - size / 2),
                 #   (x + size / 2, y + size / 2, z - size / 2),
                  #  (x - size / 2, y + size / 2, z - size / 2),
                   # (x - size / 2, y - size / 2, z + size / 2),
                  #  (x + size / 2, y - size / 2, z + size / 2),
                  #  (x + size / 2, y + size / 2, z + size / 2),
                  #  (x - size / 2, y + size / 2, z + size / 2),
               # ]

                #triangles = [
                #    (0, 1, 2), (2, 3, 0), (4, 5, 6), (6, 7, 4),
                #    (0, 1, 5), (5, 4, 0), (2, 3, 7), (7, 6, 2),
                #    (1, 2, 6), (6, 5, 1), (3, 0, 4), (4, 7, 3)
                #]
                #convex = fcl.Convex(vertices, triangles)
                #collision_object = fcl.CollisionObject(convex)

                # ? Create an FCL BVHModel (Bounding Volume Hierarchy)
                #bvh = fcl.BVHModel()
                #bvh.beginModel(len(vertices), len(triangles))
                #bvh.addSubModel(vertices, triangles)
                #bvh.endModel()
                #collision_object = fcl.CollisionObject(bvh)
                #self.fcl_octree.append(collision_object)
        #occupied_voxels=np.array(occupied_voxels,dtype=np.float32)
        #self.fcl_octree = fcl.OcTree(occupied_voxels,0.05)
        self.get_logger().info(f"? Converted {len(self.fcl_octree)} occupied nodes into FCL objects!")

    def is_collision_free2(self,point):
        self.get_logger().warn(f" inside is_collision_free")
        if(self.fcl_octree is None):
            return True
        octree_object = fcl.CollisionObject(self.fcl_octree)
        self.get_logger().warn(f" generate octree_object {octree_object}")
        # ? Define a moving sphere (radius = 0.1m)
        sphere = fcl.Sphere(0.1)
        x, y, z = point
        print(point)
        sphere_tf = fcl.Transform([x,y,z])  # Position of the sphere
        sphere_object = fcl.CollisionObject(sphere, sphere_tf)

        # ? Perform collision checking
        request = fcl.CollisionRequest()
        result = fcl.CollisionResult()
        collision = fcl.collide(octree_object, sphere_object, request, result)
        if collision:
            self.get_logger().warn(f" Collision detected between Octomap and point{x,y,z}!")
            return False
        else:
            self.get_logger().info("? No collision detected.")
            return True



    def find_nearest_node(self, query_point, find_occupied=True):
        """
        Find the nearest occupied or free node in an OctoMap.
        
        Parameters:
        - octree (octomap.OcTree): The OctoMap object.
        - query_point (tuple): The (x, y, z) coordinates to search from.
        - find_occupied (bool): If True, finds the nearest occupied node; if False, finds the nearest free node.

        Returns:
        - tuple: (nearest_node_coords, distance) or (None, None) if no valid node found.
        """
        nearest_node = None
        min_distance = 0.05

        # Iterate through all leaf nodes in the OctoMap
        for node in self.occupancy_map.begin_leafs():
            node_coord = node.getCoordinate()
            node_occupancy = node.getOccupancy()  # Occupancy probability (0=free, 1=occupied)
            
            # Check if the node matches the requested type (occupied or free)
            is_occupied = node_occupancy > 0.5  # Adjust threshold if necessary
            if is_occupied != find_occupied:
                continue

            # Compute Euclidean distance
            node_point = np.array([node_coord[0], node_coord[1], node_coord[2]])
            query_point_np = np.array(query_point)
            distance = np.linalg.norm(node_point - query_point_np)

            # Update nearest node if closer
            if distance < min_distance:
                nearest_node = node_point
                min_distance = distance

        return (tuple(nearest_node), min_distance) if nearest_node is not None else (None, None)


    def is_collision_free(self, point):
        """ Check if a point is collision-free in the Octomap. """
        if self.occupancy_map is None:
            return True  # If no map, assume free space
        
        x, y, z = point
        #print(point)

        nod = self.occupancy_map.search((x, y, z))
        if nod is None :
            return True
        else:
            nearest_node=self.find_nearest_node(point)
            self.get_logger().info(f"search node {nod},nearest_node {nearest_node[0]},steer point:{x,y,z}")
            if nearest_node[0] is None:
                return True
            else:
                self.get_logger().info(f"too close nearest_node and distance {nearest_node},steer point:{x,y,z}")
                return False


        #key = self.occupancy_map.coordToKey(np.array(point))
        #node_coords = self.occupancy_map.keyToCoord(key)
        #self.get_logger().info(f"node_coords,point: {node_coords},{x,y,z}")
        #try:
        #if nod is not None:
        #    ret=self.occupancy_map.isNodeOccupied(nod)
        #    #self.get_logger().info(f"node: {node_coords},ret occuped:{ret}")
        #    return ret
        #except Exception as e:
        #    self.get_logger().error(f"Failed to process isNodeOccupied: {e}")
        #    return False


    def rrt_planning(self, start, goal, max_iter=1000, step_size=0.05):
        """ RRT Path Planning from `start` to `goal` avoiding obstacles. """
        nodes = [start]  # RRT Tree
        parents = {start: None}  # Parent dictionary
        #self.get_logger().info(f" in rrt_planning:{parents}")
        for _ in range(max_iter):
            rand_point = self.random_point(goal)
            nearest_node = self.nearest_neighbor(rand_point, nodes)
            new_node = self.steer(nearest_node, rand_point, step_size)

            #self.get_logger().info(f" beofre is_collision_free")
            if self.is_collision_free(new_node):
                nodes.append(new_node)
                parents[new_node] = nearest_node

                if np.linalg.norm(np.array(new_node) - np.array(goal)) < step_size:
                    return self.reconstruct_path(parents, new_node)

        return None  # No valid path found

    def random_point(self, goal, bias=0.1):
        """ Generate a random point in space with a goal bias. """
        if random.random() < bias:
            return goal  # Occasionally pick the goal to bias growth
        return (
            random.uniform(self.bounds[0], self.bounds[1]),
            random.uniform(self.bounds[2], self.bounds[3]),
            random.uniform(self.bounds[4], self.bounds[5])
        )

    def nearest_neighbor(self, point, nodes):
        """ Find the nearest node in the tree using KDTree. """
        tree = KDTree(nodes)
        _, idx = tree.query(point)
        return nodes[idx]

    def steer(self, from_node, to_node, step_size):
        """ Generate a new node in the direction of `to_node` from `from_node`. """
        direction = np.array(to_node) - np.array(from_node)
        length = np.linalg.norm(direction)
        if length < step_size:
            return to_node
        direction = direction / length
        new_point = np.array(from_node) + direction * step_size
        return tuple(new_point)

    def reconstruct_path(self, parents, end_node):
        """ Reconstruct the path from the end node to the start. """
        path = []
        node = end_node
        while node:
            path.append(node)
            node = parents[node]
        path.reverse()
        return path

    def visualize_path(self, path):
        """ Publish the planned path as a Marker in RViz. """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # Line width
        marker.color.a = 1.0  # Alpha
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue

        for (x, y, z) in path:
            point = Point()
            point.x, point.y, point.z = x, y, z
            marker.points.append(point)
            self.robot.movej_xyz(z*1000,-x*1000,0,-63,50,1)

        self.path_publisher.publish(marker)
        self.get_logger().info("Published RRT path.")

    def hitbot_end_xyzr_callback(self,msg):
        xyzr=msg.data.split(",");
        self.get_logger().info(f'hitbot_end_xyzr_callback:{msg},{xyzr}')
        ret=self.robot.movel_xyz(int(xyzr[0]),int(xyzr[1]),int(xyzr[2]),int(xyzr[3]),80)
        self.get_logger().info(f"movel_xyz ret: {ret}")
        self.robot.wait_stop()

    
    

    def publish_joint_states(self):
        # Get real joint positions from HitBot API (Replace this with actual API calls)
        self.robot.get_scara_param()
        joint_positions = [self.robot.z,self.robot.angle1*3.14/180,self.robot.angle2*3.14/180,self.robot.r*3.14/180]

        # Create JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = joint_positions

        self.hitbot_x=self.robot.x/1000
        self.hitbot_y=self.robot.y/1000
        self.hitbot_z=self.robot.z/1000


        # Publish the joint states
        self.joint_state_pub.publish(joint_state_msg)

        self.publish_hitbot_x(str(int(self.robot.x)))
        self.publish_hitbot_y(str(int(self.robot.y)))
        self.publish_hitbot_z(str(int(self.robot.z)))
        self.publish_hitbot_r(str(int(self.robot.r)))
        camera_xyz=String()
        camera_xyz.data=str(self.robot.x)+","+str(self.robot.y)
        self.camera_xyz_publisher.publish(camera_xyz)
        r.set("global_camera_xy",camera_xyz.data)
        
        #self.get_logger().info(f"Published joint states: {joint_state_msg}")
    def joint_command_callback(self, msg):
        #self.get_logger().info(f"joint_command_callback trajectory: {msg}")
        #if not msg.points:
        #    return
        # Get the last trajectory point (final command position)
        #last_point = msg.points[-1]
        #joint_positions = last_point.positions  # Extract joint positions
        #hitbot_command_msg = Float64MultiArray()
        #hitbot_command_msg.data = list(joint_positions)  # Convert to list
        ##self.hitbot_command_pub.publish(hitbot_command_msg)
        try:
            positions = []
            #joint_limits = [(-1.2, 0.0), (-1.571, 1.571), (-2.967, 2.967), (-18.850, 18.850)]
            trajs=msg.trajectory[-1].joint_trajectory.points
            self.get_logger().info(f"joint_command_callback trajectory: {trajs}")
            for i in range(0, len(trajs)):
                waypoints = trajs[i].positions
                self.get_logger().info(f"i={i},waypoints : {waypoints}")
                #for j in range(0, len(waypoints)):
                  #positions=waypoints[j].positions
                self.robot.new_movej_angle(waypoints[1]*180/3.14, waypoints[2]*180/3.14, waypoints[0], waypoints[3]*180/3.14, 50, 1)
                #
            self.robot.wait_stop()

                #if pos < min_limit or pos > max_limit:
                #    print(f"Position for joint{i} must be between {min_limit} and {max_limit}.")
                #    return self.get_positions_from_user()
                #positions.append(pos)
                

            #return positions
        except ValueError:
            print("Invalid input. Please enter numerical values.")


        self.get_logger().info(f"Sent joint command to HitBot")    

    def publish_hitbot_x(self, data):
        msg = String()

        msg.data = (data)
        self.hitbot_x_publisher.publish(msg)

    def publish_hitbot_y(self, data):
        msg = String()
        msg.data = (data)

        self.hitbot_y_publisher.publish(msg)

    def publish_hitbot_z(self, data):
        msg = String()
        msg.data = (data)

        self.hitbot_z_publisher.publish(msg)

    def publish_hitbot_r(self, data):
        msg = String()
        msg.data = (data)
        self.hitbot_r_publisher.publish(msg)




    def set_gpio_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.set_digital_out(request.gpio_number, request.set_on)
                response.success = True
                self.get_logger().info('GPIO setting successful: gpio_number=%d, set_on=%r' % (request.gpio_number, request.set_on))
                break
            except Exception as e:
                self.get_logger().error('Failed to set GPIO: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying GPIO setting (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to set GPIO.')
                    break

        return response
    
    def get_gpio_out_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.get_digital_out(request.gpio_number)
                response.success = True
                if self.robot.get_digital_out(request.gpio_number) == 1:
                    self.get_logger().info('GPIO : gpio_number=%d is On' % (request.gpio_number))
                elif self.robot.get_digital_out(request.gpio_number) == 0:
                    self.get_logger().info('GPIO : gpio_number=%d is Off' % (request.gpio_number))
                elif self.robot.get_digital_out(request.gpio_number) == -1:
                    self.get_logger().info('GPIO number parameter error')
                elif self.robot.get_digital_out(request.gpio_number) == 3:
                    self.get_logger().info('GPIO Not initialized.')
                else:
                    pass
                break
            except Exception as e:
                self.get_logger().error('Failed to get GPIO: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying GPIO status (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to get GPIO.')
                    break

        return response
    
    def get_gpio_in_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.get_digital_out(request.gpio_number)
                response.success = True
                if self.robot.get_digital_out(request.gpio_number) == 1:
                    self.get_logger().info('GPIO : gpio_number=%d is triggered' % (request.gpio_number))
                elif self.robot.get_digital_out(request.gpio_number) == 0:
                    self.get_logger().info('GPIO : gpio_number=%d is not triggered' % (request.gpio_number))
                elif self.robot.get_digital_out(request.gpio_number) == -1:
                    self.get_logger().info('GPIO number parameter error')
                elif self.robot.get_digital_out(request.gpio_number) == 3:
                    self.get_logger().info('GPIO Not initialized.')
                else:
                    pass
                break
            except Exception as e:
                self.get_logger().error('Failed to get GPIO: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying GPIO status (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to get GPIO.')
                    break

        return response

    def set_drag_teach_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.set_drag_teach(request.enable)
                response.success = True
                if self.robot.set_drag_teach(request.enable) == True:
                    self.get_logger().info('Successfully set drag teach')
                elif self.robot.set_drag_teach(request.enable) == False:
                    self.get_logger().info('Failed to set drag teach, please check your robot model enable drag teach')
                else:
                    pass
                break
            except Exception as e:
                self.get_logger().error('Failed to set drag teach: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying set drag teach (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to set drag teach.')
                    break

        return response

    def joint_home_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.joint_home(request.joint_num)
                response.success = True
                if self.robot.joint_home(request.joint_num) == 0:
                    self.get_logger().info('Joint Not connected. Check your robot')
                elif self.robot.joint_home(request.joint_num) == 1:
                    self.get_logger().info('Success')
                elif self.robot.joint_home(request.joint_num) == 2:
                    self.get_logger().info('Invalid parameters passed')
                elif self.robot.joint_home(request.joint_num) == 3:
                    self.get_logger().info('The robot is still initializing.')
                else:
                    pass
                break
            except Exception as e:
                self.get_logger().error('Failed to joint home: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying joint home (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to joint home.')
                    break
        return response

    def new_movej_xyz_lr_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.new_movej_xyz_lr(request.goal_x, request.goal_y, request.goal_z, request.goal_r, request.speed, request.roughly, request.lr)
                response.success = True
                if self.robot.new_movej_xyz_lr(request.goal_x, request.goal_y, request.goal_z, request.goal_r, request.speed, request.roughly, request.lr) == 1:
                    self.get_logger().info('Robot Moving')
                else:
                    self.get_logger().info('Error, Check Robot')
                break
            except Exception as e:
                self.get_logger().error('Failed to new_movej_xyz_lr: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying new_movej_xyz_lr (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to new_movej_xyz_lr.')
        return response


    def new_movej_angle_callback(self, request, response):
        max_retries = 3
        retries = 0
        
        while retries < max_retries:
            try:
                self.robot.movej_angle(request.goal_angle1, request.goal_angle2, request.goal_z, request.goal_r, request.speed, request.roughly)
                response.success = True
                if self.robot.movej_angle(request.goal_angle1, request.goal_angle2, request.goal_z, request.goal_r, request.speed, request.roughly) == 1:
                    self.get_logger().info('Robot Moving')
                else:
                    self.get_logger().info('Error, Check Robot')
                break
            except Exception as e:
                self.get_logger().error('Failed to new_movej_angle: %s' % str(e))
                response.success = False
                retries += 1
                if retries < max_retries:
                    self.get_logger().info('Retrying new_movej_angle (attempt %d)...' % retries)
                else:
                    self.get_logger().error('Max retries exceeded. Failed to new_movej_angle.')
        return response

    def init_robot(self):
        self.robot.net_port_initial()
        time.sleep(1)
        is_connected = self.robot.is_connect()

        if is_connected != 1:
            print('No robot connection!!!')
            raise RuntimeError('No robot connection!!!')

        print('Robot connected.')

        init = self.robot.initial(1, 210) ## 1000 is z-axis parameter, Modify it to your own

        if init != 1:
            print('Robot initialization failed!!!')
            raise RuntimeError('Robot initialization failed!!!')

        print('unlock Robot')
        self.robot.unlock_position()
        print('Robot position initialized.')
        ret=self.robot.movel_xyz(600,0,0,-63,20)
    	#ret=self.robot.new_movej_xyz_lr(hi.x-100,hi.y,0,-63,20,0,1)
        #ret=self.robot.movej_angle(0,30,0,0,20,0)
        self.robot.wait_stop()
        print('Robot I/O output initialized.')
        for i in range(12):
            self.robot.set_digital_out(i, False)
        time.sleep(1)
        print('Robot initialized.')

    def run(self):
        print("hello hibot")

        while rclpy.ok():
            try:
                rclpy.spin_once(self)
            except ValueError as e:
                print("Error:", str(e))
            except RuntimeError as e:
                print("Error:", str(e))

        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    hitbot_controller = HitbotController()

    try:
        hitbot_controller.run()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        hitbot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
