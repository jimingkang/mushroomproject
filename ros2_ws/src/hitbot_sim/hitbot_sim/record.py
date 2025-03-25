import h5py
import numpy as np
import os
import time
from collections import deque
import rclpy
import time
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Int64,String
from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import DisplayTrajectory

from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge

class SCARA_ACT_Recorder(Node):
    def __init__(self, dataset_dir):
        super().__init__('SCARA_ACT_Recorder')
        self.dataset_dir = dataset_dir
        os.makedirs(dataset_dir, exist_ok=True)
        
        # SCARA configuration
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.gripper_joint = 'joint4'  # Assuming last joint controls gripper
        
        # Data buffers
        self.buffer = {
            'observations': {
                'images': deque(maxlen=1000),
                'qpos': deque(maxlen=1000),
                'cartesian_pos': deque(maxlen=1000),
                'gripper': deque(maxlen=1000)
            },
            'action': deque(maxlen=1000),
            'timestamps': deque(maxlen=1000)
        }
        
        # ROS setup
        self.bridge = CvBridge()
        self.joints_sub=self.create_subscription( JointState,"/hitbot/joint_states", self.joint_state_cb,10)
        self.image_sub=self.create_subscription(Image,"/camera/camera/color/image_rect_raw",  self.image_cb,10)
        
        self.recording = False
        self.episode_count = 0
        self.last_qpos = None

    def joint_state_cb(self, msg):
        if not self.recording:
            return
            
        # Extract joint positions in correct order
        qpos = np.zeros(4)
        for i, name in enumerate(self.joint_names):
            idx = msg.name.index(name)
            qpos[i] = msg.position[idx]
        
        # Compute cartesian position (forward kinematics)
        cartesian_pos = self.forward_kinematics(qpos)
        self.get_logger().info(f"cartesian_pos:{cartesian_pos}")

        
        # Compute action (delta from last position)
        if self.last_qpos is not None:
            action = qpos - self.last_qpos
            self.buffer['action'].append(action)
        
        # Store observations
        self.buffer['observations']['qpos'].append(qpos)
        self.buffer['observations']['cartesian_pos'].append(cartesian_pos)
        self.buffer['observations']['gripper'].append(qpos[-1])  # Last joint is gripper
        self.buffer['timestamps'].append(msg.header.stamp.to_sec())
        
        self.last_qpos = qpos

    def image_cb(self, msg):
        if not self.recording:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.buffer['observations']['images'].append(cv_image)
        except Exception as e:
            rclpy.logerr(e)

    def forward_kinematics(self, qpos):
        """Simple SCARA forward kinematics"""
        θ1, θ2, d, θ4 = qpos
        x = 0.325 * np.cos(θ1) + 0.275 * np.cos(θ2)+0.17 * np.cos(θ4)  # Modify with your arm's lengths
        y = 0.325 * np.sin(θ1) + 0.275 * np.sin(θ2)+0.17 * np.sin(θ4)
        z = d  # Prismatic joint
        rz = θ1 + θ2 + θ4  # Total rotation
        return np.array([x, y, z, rz])

    def start_recording(self):
        self.recording = True
        self.last_qpos = None
        self.buffer = {k: {sk: deque(maxlen=1000) for sk in v} if isinstance(v, dict) else deque(maxlen=1000) 
                     for k, v in self.buffer.items()}
        self.get_logger().info("Started recording new episode")

    def stop_and_save(self):
        if not self.recording:
            return
            
        self.recording = False
        
        # Ensure all buffers have same length
        min_length = min(len(self.buffer['observations']['images']),
                        len(self.buffer['observations']['qpos']),
                        len(self.buffer['action']))
        
        # Trim buffers
        for k in self.buffer['observations']:
            self.buffer['observations'][k] = list(self.buffer['observations'][k])[:min_length]
        self.buffer['action'] = list(self.buffer['action'])[:min_length]
        self.buffer['timestamps'] = list(self.buffer['timestamps'])[:min_length]
        
        # Save to HDF5
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.dataset_dir, f"episode_{self.episode_count:06d}_{timestamp}.hdf5")
        
        with h5py.File(filename, 'w') as f:
            # Create observations group
            obs_group = f.create_group('observations')
            obs_group.create_dataset('images0', 
                                  data=np.array(self.buffer['observations']['images']),
                                  compression='gzip')
            obs_group.create_dataset('qpos',
                                  data=np.array(self.buffer['observations']['qpos']),
                                  compression='gzip')
            obs_group.create_dataset('cartesian_pos',
                                  data=np.array(self.buffer['observations']['cartesian_pos']),
                                  compression='gzip')
            obs_group.create_dataset('gripper',
                                  data=np.array(self.buffer['observations']['gripper']),
                                  compression='gzip')
            
            # Save actions and timestamps
            f.create_dataset('action',
                           data=np.array(self.buffer['action']),
                           compression='gzip')
            f.create_dataset('timestamps',
                           data=np.array(self.buffer['timestamps']),
                           compression='gzip')
            
            # Metadata
            f.attrs['robot_type'] = 'SCARA'
            f.attrs['joint_names'] = self.joint_names
            f.attrs['episode_id'] = self.episode_count
            
        self.episode_count += 1
        self.get_logger().info(f"Saved episode {filename}")

def main(args=None):
    rclpy.init(args=args)
    recorder = SCARA_ACT_Recorder("scara_act_dataset")
    try:
        input()
        recorder.start_recording()
        print("Recording... Press Enter to stop")
        input()
        recorder.stop_and_save()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    rclpy.init()
    recorder = SCARA_ACT_Recorder("scara_act_dataset")
    
    print("SCARA ACT Recorder Ready")
    print("Press Enter to start recording...")
    input()
    recorder.start_recording()
    
    print("Recording... Press Enter to stop")
    input()
    recorder.stop_and_save()