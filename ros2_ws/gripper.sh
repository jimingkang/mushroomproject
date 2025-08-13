source ~/.bashrc
export ROS_DOMAIN_ID=5
source /opt/ros/humble/setup.bash
cd ~/Downloads/mushroomproject/ros2_ws/
source  install/setup.bash
CMD="ps -ef|grep hitbot_sim |wc -l"
eval $CMD
    if [ $? -gt 1 ]; 
    then 
        echo "\"$CMD\" executed successfully" 
    else 
        echo "\"$CMD\" terminated unsuccessfully, return code $?" 
	nohup ros2 run  hitbot_sim hitbot_controller_joint_state > gripper.log &
    fi 
