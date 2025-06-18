cd ~/Downloads/mushroomproject/ros2_ws/
source install/setup.bash
nohup ros2 launch realsense2_camera rs_launch.py     rgb_camera.profile:=848x480x15     depth_module.profile:=848x480x15     align_depth.enable:=true >camera.log &
nohup ros2 launch yolox_ros_py yolox_nano_torch_gpu_camera.launch.py > yolox_ros.log &
nohup ros2 run  hitbot_sim hitbot_controller_joint_state --ckpt_dir ./2  --policy_class ACT --task_name sim_pick_n_place_scripted --batch_size 1  --seed 0 --num_epochs 100 > hitbot_sim.log &
