#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "opencv2/contrib/contrib.hpp"
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
nav_msgs::Odometry odom;
tf::Pose old_pose, new_pose, trans;
cv_bridge::CvImagePtr cv_ptr_old;
boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_l;
boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_r;
image_transport::CameraPublisher image_pub_1;
image_transport::CameraPublisher image_pub_2;
// parameters
std::string image_topic_, odom_topic_, camera_name_, camera_info_url_;
int image_width_, image_height_;
bool flag = false;
bool flag_1 = false;
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

    //tf::Vector3 tl(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    tf::Vector3 tl(-odom.pose.pose.position.y, -odom.pose.pose.position.z, odom.pose.pose.position.x);
    tf::Vector3 position;
    
    tf::Quaternion qt(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);
    tf::Quaternion qt_trans;
    qt_trans.setRPY(-pitch,-yaw,roll);
 
//     tf::TransformListener listener;
//     tf::StampedTransform transform;
//     try{
// listener.waitForTransform("/openni_camera", "/world", ros::Time(0), ros::Duration(2.0) );
//       listener.lookupTransform("/openni_camera","/world",  
//                                ros::Time(0), transform);
//     }
//     catch (tf::TransformException ex){
//       ROS_ERROR("%s",ex.what());
//       ros::Duration(0.2).sleep();
//     }
//     new_pose.setOrigin(transform.getOrigin());
//     new_pose.setRotation(transform.getRotation());

    new_pose.setOrigin(tl);
    new_pose.setRotation(qt_trans);
    if (!flag_1){
      old_pose = new_pose;
      ROS_INFO("old_pose init!");
      flag_1 = true;
    }
    trans = new_pose.inverseTimes(old_pose);

    tf::Matrix3x3 tf_R = trans.getBasis();
    tf::Vector3 tf_T = trans.getOrigin();
    double r[3][3] = {{tf_R[0][0], tf_R[0][1], tf_R[0][2]}, {tf_R[1][0], tf_R[1][1], tf_R[1][2]}, {tf_R[2][0], tf_R[2][1], tf_R[2][2]}};
    double t[3][1] = {{tf_T[0]}, {tf_T[1]}, {tf_T[2]}};
    double cm1[3][3] = {{ci_l->K[0], ci_l->K[1], ci_l->K[2]}, {ci_l->K[3], ci_l->K[4], ci_l->K[5]}, {ci_l->K[6], ci_l->K[7], ci_l->K[8]}};
    double cm2[3][3] = {{ci_r->K[0], ci_r->K[1], ci_r->K[2]}, {ci_r->K[3], ci_r->K[4], ci_r->K[5]}, {ci_r->K[6], ci_r->K[7], ci_r->K[8]}};
    double d1[1][5] = {{ ci_l->D[0], ci_l->D[1], ci_l->D[2], ci_l->D[3], ci_l->D[4]}};
    double d2[1][5] = {{ ci_r->D[0], ci_r->D[1], ci_r->D[2], ci_r->D[3], ci_r->D[4]}};
    
    cv::Mat CM1(3,3, CV_64FC1, cm1);
    cv::Mat CM2(3,3, CV_64FC1, cm2);
    cv::Mat D1(1,5, CV_64FC1, d1);
    cv::Mat D2(1,5, CV_64FC1, d2);
    cv::Mat R(3,3, CV_64FC1, r);
    cv::Mat T(3,1, CV_64FC1, t);
    cv::Mat R1, R2, P1, P2, Q;
	  cv::Size s(image_width_,image_height_);

    //tf::Vector3 tl2 = old_pose.getOrigin();
    //ROS_INFO("Old Pos %f %f %f",tl2[0],tl2[1],tl2[2]);
    //ROS_INFO("New Pos %f %f %f",tl[0],tl[1],tl[2]);
    position = trans.getOrigin();
    //ROS_INFO("Transform %f %f %f",position[0],position[1],position[2]);
    if ((std::abs(position[0])) > 0.1f && (std::abs(position[0]) + std::abs(position[1]) + std::abs(position[2]))!= 0)
    {
      // std::cout << "CM1 = "<< std::endl << " "  << CM1 << std::endl << std::endl;
      // std::cout << "CM2 = "<< std::endl << " "  << CM2 << std::endl << std::endl;
      // std::cout << "D1 = "<< std::endl << " "  << D1 << std::endl << std::endl;
      // std::cout << "D2 = "<< std::endl << " "  << D2 << std::endl << std::endl;
      // std::cout << "R = "<< std::endl << " "  << R << std::endl << std::endl;
      // std::cout << "T = "<< std::endl << " "  << T << std::endl << std::endl;   
      cv::stereoRectify(CM1, D1, CM2, D2, s, R, T, R1, R2, P1, P2, Q);
      // std::cout << "P1 = "<< std::endl << " "  << P1 << std::endl << std::endl;
      // std::cout << "P2 = "<< std::endl << " "  << P2 << std::endl << std::endl;
      // std::cout << "R1 = "<< std::endl << " "  << R1 << std::endl << std::endl;
      // std::cout << "R2 = "<< std::endl << " "  << R2 << std::endl << std::endl;
      ci_l->R[0] = R1.at<double>(0, 0);
      ci_l->R[1] = R1.at<double>(0, 1);
      ci_l->R[2] = R1.at<double>(0, 2);
      ci_l->R[3] = R1.at<double>(1, 0);
      ci_l->R[4] = R1.at<double>(1, 1);
      ci_l->R[5] = R1.at<double>(1, 1);
      ci_l->R[6] = R1.at<double>(2, 0);
      ci_l->R[7] = R1.at<double>(2, 1);
      ci_l->R[8] = R1.at<double>(2, 2);

      ci_r->R[0] = R2.at<double>(0, 0);
      ci_r->R[1] = R2.at<double>(0, 1);
      ci_r->R[2] = R2.at<double>(0, 2);
      ci_r->R[3] = R2.at<double>(1, 0);
      ci_r->R[4] = R2.at<double>(1, 1);
      ci_r->R[5] = R2.at<double>(1, 1);
      ci_r->R[6] = R2.at<double>(2, 0);
      ci_r->R[7] = R2.at<double>(2, 1);
      ci_r->R[8] = R2.at<double>(2, 2);

      ci_l->P[0] = P1.at<double>(0, 0);
      ci_l->P[1] = P1.at<double>(0, 1);
      ci_l->P[2] = P1.at<double>(0, 2);
      ci_l->P[3] = P1.at<double>(0, 3);
      ci_l->P[4] = P1.at<double>(1, 0);
      ci_l->P[5] = P1.at<double>(1, 1);
      ci_l->P[6] = P1.at<double>(1, 2);
      ci_l->P[7] = P1.at<double>(1, 3);
      ci_l->P[8] = P1.at<double>(2, 0);
      ci_l->P[9] = P1.at<double>(2, 1);
      ci_l->P[10] = P1.at<double>(2, 2);
      ci_l->P[11] = P1.at<double>(2, 3);

      ci_r->P[0] = P2.at<double>(0, 0);
      ci_r->P[1] = P2.at<double>(0, 1);
      ci_r->P[2] = P2.at<double>(0, 2);
      ci_r->P[3] = P2.at<double>(0, 3);
      ci_r->P[4] = P2.at<double>(1, 0);
      ci_r->P[5] = P2.at<double>(1, 1);
      ci_r->P[6] = P2.at<double>(1, 2);
      ci_r->P[7] = P2.at<double>(1, 3);
      ci_r->P[8] = P2.at<double>(2, 0);
      ci_r->P[9] = P2.at<double>(2, 1);
      ci_r->P[10] = P2.at<double>(2, 2);
      ci_r->P[11] = P2.at<double>(2, 3);
      image_pub_1.publish(msg, ci_l);
      if (flag)
        image_pub_2.publish(cv_ptr_old->toImageMsg(), ci_r);
      old_pose = new_pose;
      //ROS_INFO("Old pose set");
      cv_ptr_old = cv_bridge::toCvCopy(msg);
      flag = true;
    }

    cv::waitKey(30);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("%s",e.what());
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
