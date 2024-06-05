#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>


//#include "sensor_msgs/PointCloud.h"
//#include "geometry_msgs/PoseStamped.h"
//#include "geometry_msgs/Point32.h"
//#include "tf/transform_datatypes.h"
//#include <tf/transform_broadcaster.h>

//using namespace std;
//using namespace tf;

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
   ,m_SLAM(pSLAM)
{
     std::cout << "camera/camera/color/image_rect_raw subscribed" << std::endl;
    
    //rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/camera/camera/color/image_rect_raw");
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/color/image_rect_raw");
    
   std::cout << "slam changed" << std::endl;
    
   // depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/camera/camera/depth/image_rect_raw");
 depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/depth/image_rect_raw");
   syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

   //    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/slam/pointcloud", 1);
   // ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/slam/pos", 1);
//syncApproximate.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2, pos_pub, cloud_pub));
 
std::cout << "RgbdSlamNode()" << std::endl;
}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));
  
}
/*
RgbdSlamNode::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, ros::Publisher pos_pub, ros::Publisher cloud_pub)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = "world";
    std::vector<geometry_msgs::Point32> geo_points;
    std::vector<ORB_SLAM2::MapPoint*> points = m_SLAM->GetTrackedMapPoints();
    cout << points.size() << endl;
    for (std::vector<int>::size_type i = 0; i != points.size(); i++) {
	    if (points[i]) {
		    cv::Mat coords = points[i]->GetWorldPos();
		    geometry_msgs::Point32 pt;
		    pt.x = coords.at<float>(0);
		    pt.y = coords.at<float>(1);
		    pt.z = coords.at<float>(2);
		    geo_points.push_back(pt);
	    } else {
	    }
    }
    cout << geo_points.size() << endl;
    cloud.points = geo_points;
    cloud_pub.publish(cloud);
}
*/
