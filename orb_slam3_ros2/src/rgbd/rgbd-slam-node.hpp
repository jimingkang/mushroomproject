#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/path.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point32.hpp"
//#include "tf/transform_datatypes.h"
//#include <tf/transform_broadcaster.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class RgbdSlamNode : public rclcpp::Node
{
public:

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_camerapath;
    nav_msgs::msg::Path  camerapath;

    // rclcpp::Publisher<geometry_msgs::PoseStamped>::SharedPtr CamPose_Pub;
     //rclcpp::Publisher<geometry_msgs::PoseWithCovarianceStamped>::SharedPtr Camodom_Pub; 
    // image_transport::Publisher tracking_img_pub;
   // rclcpp::Publisher<sensor_msgs::PointCloud2>::SharedPtr tracked_mappoints_pub; 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  all_mappoints_pub;
        

    RgbdSlamNode(ORB_SLAM3::System* pSLAM);

    ~RgbdSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);
 //void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, ros::Publisher pos_pub, ros::Publisher cloud_pub);
    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > depth_sub;
      std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;
  
};

#endif
