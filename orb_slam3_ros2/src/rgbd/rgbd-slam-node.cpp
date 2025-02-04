#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>



#include <pcl_conversions/pcl_conversions.h>



using namespace std;
//using namespace tf;
using namespace nav_msgs;
using namespace geometry_msgs;
using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
   ,m_SLAM(pSLAM)
{
    //原文链接：https://blog.csdn.net/qq_41694024/article/details/134669040
        pub_camerapath=this->create_publisher<nav_msgs::msg::Path> ("/Path", 100);

   // CamPose_Pub =this->create_publisher<geometry_msgs::PoseStamped>("/ORB_SLAM3/Camera_Pose",100);
  //  Camodom_Pub =this->create_publisher<geometry_msgs::PoseWithCovarianceStamped>("/ORB_SLAM3/Camera_Odom", 100);
 // tracking_img_pub = image_transport::create_publisher( "/ORB_SLAM3/tracking_image", 1);
 //   tracked_mappoints_pub = this->create_publisher<sensor_msgs::PointCloud2>( "/ORB_SLAM3/tracked_pointcloud");
    all_mappoints_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ORB_SLAM3/pointclouds",1);
                        

     std::cout << "/camera/camera/color/image_rect_raw subscribed" << std::endl;
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/color/image_rect_raw"); //
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
 boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> pcl_cloudPTR(new pcl::PointCloud<pcl::PointXYZRGBA>);
 sensor_msgs::msg::PointCloud2 cloud2;

     m_SLAM->getPointCloudMap(pcl_cloudPTR);
      pcl::toROSMsg(*pcl_cloudPTR,cloud2);
      cout<<"show getPointCloudMap before published, size="<<pcl_cloudPTR->points.size()<<endl;
    this->all_mappoints_pub->publish(cloud2);
/*
    geometry_msgs::Pose sensor_pose;
    Sophus::SE3f sophus_Tcw;
    sophus_Tcw = m_SLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    cv::Mat Tcw = cv::Mat(4, 4, CV_32F);
    cv::eigen2cv(sophus_Tcw.matrix(), Tcw);
 
    orb_slam_broadcaster = new tf::TransformBroadcaster;
 
    // 3.发布正在追踪的图像(有特征点哪个)
    //publish_tracking_img(m_SLAM->GetCurrentFrame(), cv_ptrRGB->header.stamp);
 
    // 4.发布正在追踪的地图点
    //this->tracked_mappoints_pub.publish(m_SLAM->GetTrackedMapPoints());
 
    // 5.发布所有的地图点
   
    if (!Tcw.empty())
    {
        cv::Mat Twc =Tcw.inv();
        cv::Mat RWC= Twc.rowRange(0,3).colRange(0,3);
        cv::Mat tWC=  Twc.rowRange(0,3).col(3);
        cv::Mat twc(3,1,CV_32F);
        twc = tWC;
        Eigen::Matrix<double,3,3> eigMat ;
        eigMat <<RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
                RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
                RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2);
        Eigen::Quaterniond q(eigMat);
 
        Pose_quat[0] = q.x(); Pose_quat[1] = q.y();
        Pose_quat[2] = q.z(); Pose_quat[3] = q.w();
 
        Pose_trans[0] = twc.at<float>(0);
        //Pose_trans[1] = twc.at<float>(1);
        Pose_trans[1] = 0;
        Pose_trans[2] = twc.at<float>(2);
 
         6.sensor_pose存放camera的位姿
        sensor_pose.position.x = twc.at<float>(0);
        sensor_pose.position.y = twc.at<float>(1);
        sensor_pose.position.z = twc.at<float>(2);
        sensor_pose.orientation.x = q.x();
        sensor_pose.orientation.y = q.y();
        sensor_pose.orientation.z = q.z();
        sensor_pose.orientation.w = q.w();
 
        m_SLAM->setOrigin(tf::Vector3(Pose_trans[2], -Pose_trans[0], -Pose_trans[1]));
        m_SLAM->setRotation(tf::Quaternion(q.z(), -q.x(), -q.y(), q.w()));
        orb_slam_broadcaster->sendTransform(tf::StampedTransform(*m_SLAM, ros::Time::now(), "/odom", "/orb_cam_link"));
 
        Cam_Pose.header.stamp =ros::Time::now();
        //Cam_Pose.header.seq = msgRGB->header.seq;
        Cam_Pose.header.frame_id = "/odom";
        tf::pointTFToMsg(orb_slam.getOrigin(), Cam_Pose.pose.position);
        tf::quaternionTFToMsg(orb_slam.getRotation(), Cam_Pose.pose.orientation);
 
        Cam_odom.header.stamp = ros::Time::now();
        Cam_odom.header.frame_id = "/odom";
        tf::pointTFToMsg(orb_slam.getOrigin(), Cam_odom.pose.pose.position);
        tf::quaternionTFToMsg(orb_slam.getRotation(), Cam_odom.pose.pose.orientation);
        Cam_odom.pose.covariance = {0.01, 0, 0, 0, 0, 0,
                                    0, 0.01, 0, 0, 0, 0,
                                    0, 0, 0.01, 0, 0, 0,
                                    0, 0, 0, 0.01, 0, 0,
                                    0, 0, 0, 0, 0.01, 0,
                                    0, 0, 0, 0, 0, 0.01};
 
        // 7.发布相机单帧位姿
        CamPose_Pub.publish(Cam_Pose);
        //Camodom_Pub.publish(Cam_odom);
        std_msgs::Header header ;
        header.stamp =msgRGB->header.stamp;
        header.seq = msgRGB->header.seq;
        header.frame_id="/odom";
        camerapath.header =header;
        camerapath.poses.push_back(Cam_Pose);
 
        //7.发布相机轨迹
        pub_camerapath.publish(camerapath);
  */
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
