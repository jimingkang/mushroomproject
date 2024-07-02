import pyrealsense2 as rs
import numpy as np
import json

# JSON configuration string
json_string = '''
{
    "device": {
        "fw version": "05.14.00.00",
        "name": "Intel RealSense D405",
        "product line": "D400"
    },
    "parameters": {
        "aux-param-autoexposure-setpoint": "1000",
        "aux-param-colorcorrection1": "-0.0556641",
        "aux-param-colorcorrection10": "-0.194336",
        "aux-param-colorcorrection11": "-0.194336",
        "aux-param-colorcorrection12": "-0.589844",
        "aux-param-colorcorrection2": "0.560547",
        "aux-param-colorcorrection3": "0.560547",
        "aux-param-colorcorrection4": "0.170898",
        "aux-param-colorcorrection5": "-0.275391",
        "aux-param-colorcorrection6": "-0.238281",
        "aux-param-colorcorrection7": "-0.238281",
        "aux-param-colorcorrection8": "1.34766",
        "aux-param-colorcorrection9": "0.959961",
        "aux-param-depthclampmax": "65536",
        "aux-param-depthclampmin": "0",
        "aux-param-disparityshift": "0",
        "controls-autoexposure-auto": "False",
        "controls-autoexposure-manual": "9524",
        "controls-depth-gain": "16",
        "controls-depth-white-balance-auto": "True",
        "ignoreSAD": "0",
        "param-amplitude-factor": "0",
        "param-autoexposure-setpoint": "1000",
        "param-censusenablereg-udiameter": "9",
        "param-censusenablereg-vdiameter": "9",
        "param-censususize": "9",
        "param-censusvsize": "9",
        "param-depthclampmax": "65536",
        "param-depthclampmin": "0",
        "param-depthunits": "100",
        "param-disableraucolor": "0",
        "param-disablesadcolor": "0",
        "param-disablesadnormalize": "0",
        "param-disablesloleftcolor": "0",
        "param-disableslorightcolor": "1",
        "param-disparitymode": "0",
        "param-disparityshift": "0",
        "param-lambdaad": "751",
        "param-lambdacensus": "6",
        "param-leftrightthreshold": "526",
        "param-maxscorethreshb": "2048",
        "param-medianthreshold": "502",
        "param-minscorethresha": "0",
        "param-neighborthresh": "0",
        "param-raumine": "5",
        "param-rauminn": "3",
        "param-rauminnssum": "7",
        "param-raumins": "2",
        "param-rauminw": "2",
        "param-rauminwesum": "10",
        "param-regioncolorthresholdb": "0.785714",
        "param-regioncolorthresholdg": "0.565558",
        "param-regioncolorthresholdr": "0.704501",
        "param-regionshrinku": "3",
        "param-regionshrinkv": "0",
        "param-robbinsmonrodecrement": "10",
        "param-robbinsmonroincrement": "5",
        "param-rsmdiffthreshold": "1.75",
        "param-rsmrauslodiffthreshold": "1.40625",
        "param-rsmremovethreshold": "0.672619",
        "param-scanlineedgetaub": "13",
        "param-scanlineedgetaug": "15",
        "param-scanlineedgetaur": "30",
        "param-scanlinep1": "155",
        "param-scanlinep1onediscon": "160",
        "param-scanlinep1twodiscon": "59",
        "param-scanlinep2": "190",
        "param-scanlinep2onediscon": "507",
        "param-scanlinep2twodiscon": "493",
        "param-secondpeakdelta": "0",
        "param-texturecountthresh": "0",
        "param-texturedifferencethresh": "0",
        "param-usersm": "1",
        "param-zunits": "100"
    },
    "schema version": 1,
    "viewer": {
        "stream-depth-format": "Z16",
        "stream-fps": "30",
        "stream-height": "720",
        "stream-width": "1280"
    }
}
'''

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Replace with appropriate width, height, and fps
stream_width = 1280
stream_height = 720
stream_fps = 30

config.enable_stream(rs.stream.depth, stream_width, stream_height, rs.format.z16, stream_fps)
config.enable_stream(rs.stream.color, stream_width, stream_height, rs.format.bgr8, stream_fps)
cfg = pipeline.start(config)

# Get the device and set it to advanced mode
dev = cfg.get_device()
advnc_mode = rs.rs400_advanced_mode(dev)

# Load the JSON settings
#advnc_mode.load_json(json_string)

def capture_frames(num_frames=10):
    rgb_frames = []
    depth_frames = []

    for _ in range(num_frames):
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert the frames to NumPy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        rgb_frames.append(color_image)
        depth_frames.append(depth_image)

    # Save the frames as single NumPy files
    np.save('rgb_frames.npy', np.array(rgb_frames))
    np.save('depth_frames.npy', np.array(depth_frames))

    # Stop the camera pipeline
    pipeline.stop()

if __name__ == '__main__':
    capture_frames()
