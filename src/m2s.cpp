#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>

nav_msgs::Odometry odom;
tf::Pose old_pose, new_pose, trans;
cv_bridge::CvImagePtr cv_ptr_old;
boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_l;
boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_r;
image_transport::CameraPublisher image_pub_1;
image_transport::CameraPublisher image_pub_2;
bool flag = false;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //  grab the camera info
    sensor_msgs::CameraInfoPtr ci_l(new sensor_msgs::CameraInfo(cinfo_l->getCameraInfo()));
    ci_l->header.frame_id = msg->header.frame_id;
    ci_l->header.stamp = msg->header.stamp;
    
    sensor_msgs::CameraInfoPtr ci_r(new sensor_msgs::CameraInfo(cinfo_r->getCameraInfo()));
    ci_r->header.frame_id = msg->header.frame_id;
    ci_r->header.stamp = msg->header.stamp;

    tf::Vector3 tl(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    tf::Vector3 position;
    
    tf::Quaternion qt(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    
    new_pose.setOrigin(tl);
    new_pose.setRotation(qt);
 
    trans = new_pose.inverseTimes(old_pose);
    position = trans.getOrigin();
    
    ci_r->P[3] = (position[1])*ci_r->P[0];//-fx*baseline_x w.r.t. image_frame
    ci_r->P[7] = (position[2])*ci_r->P[5];//-fy*baseline_y
    
    if ((std::abs(position[1])) > 0.1f)
    {
      ROS_INFO("Baseline: %f",-position[1]);
      image_pub_1.publish(msg, ci_l);
      if (flag)
        image_pub_2.publish(cv_ptr_old->toImageMsg(), ci_r);
      old_pose = new_pose;
      cv_ptr_old = cv_bridge::toCvCopy(msg);
      flag = true;
    }
  

    cv::waitKey(30);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom.pose.pose = msg->pose.pose;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mono2stereo");
  ros::NodeHandle nh("~");
  //cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);

  // parameters
  std::string image_topic_, odom_topic_, camera_name_, camera_info_url_;
  int image_width_, image_height_;

  //nh.param("camera_frame_id", img_.header.frame_id, std::string("stereo"));
  nh.param("image_topic",image_topic_,std::string("/camera/image_raw"));
  nh.param("odom_topic",odom_topic_,std::string("/bebop/odom"));
  nh.param("camera_name", camera_name_, std::string("camera"));
  nh.param("camera_info_url", camera_info_url_, std::string(""));
  nh.param("image_width", image_width_, 640);
  nh.param("image_height", image_height_, 480);
  ROS_INFO("camera: %s",camera_info_url_.c_str());
  
  image_transport::Subscriber sub = it.subscribe(image_topic_, 1, imageCallback);
  image_pub_1 = it.advertiseCamera("/camera/left/image_raw", 1);
  image_pub_2 = it.advertiseCamera("/camera/right/image_raw", 1);
  ros::Subscriber pose = nh.subscribe(odom_topic_, 1, poseCallback);

  cinfo_l.reset(new camera_info_manager::CameraInfoManager(nh, camera_name_, camera_info_url_));
  cinfo_r.reset(new camera_info_manager::CameraInfoManager(nh, camera_name_, camera_info_url_));

  // check for default camera info
  if (!cinfo_l->isCalibrated())
  {
    cinfo_l->setCameraName(camera_name_);
    sensor_msgs::CameraInfo camera_info_l;
    camera_info_l.width = image_width_;
    camera_info_l.height = image_height_;
    cinfo_l->setCameraInfo(camera_info_l);
  }
  if (!cinfo_r->isCalibrated())
  {
    cinfo_r->setCameraName(camera_name_);
    sensor_msgs::CameraInfo camera_info_r;
    camera_info_r.width = image_width_;
    camera_info_r.height = image_height_;
    cinfo_r->setCameraInfo(camera_info_r);
  }

  ros::spin();
  //cv::destroyWindow("view");
}