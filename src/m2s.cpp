#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <stereo_msgs/DisparityImage.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <image_geometry/stereo_camera_model.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "opencv2/contrib/contrib.hpp"
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <png++/png.hpp>
#include "SPSStereo.h"
#include "defParameter.h"
nav_msgs::Odometry odom;
tf::Pose old_pose, new_pose, trans;
cv_bridge::CvImagePtr cv_ptr_old;
boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_l;
boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_r;
image_transport::CameraPublisher image_pub_1;
image_transport::CameraPublisher image_pub_2;
ros::Publisher pub_depth_image;
image_geometry::StereoCameraModel model_;
cv::Mat_<cv::Vec3f> points_mat_; // scratch buffer
stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage>();
// parameters
std::string image_topic_, odom_topic_, camera_name_, camera_info_url_;
int image_width_, image_height_;
bool flag = false;
bool flag_1 = false;
SPSStereo sps;

void ConvertDispImageToCvMat8(const png::image<png::gray_pixel_16>& img, cv::Mat* cvimg) {
  cvimg->create(img.get_height(), img.get_width(), CV_8U);
  for (int i = 0; i < cvimg->rows; i++)
    for (int j = 0; j < cvimg->cols; j++)
      cvimg->at<uint8_t>(i,j) = (uint8_t)std::min(255.0, std::round(img.get_pixel(j,i) / 256.0));
}

void ConvertMatToPNG(const cv::Mat& cvimg, png::image<png::rgb_pixel>* img) {
  for (size_t i = 0; i < img->get_height(); i++) {
    for (size_t j = 0; j < img->get_width(); j++) {
      png::rgb_pixel pix;
      cv::Vec3b intensity = cvimg.at<cv::Vec3b>(i,j);
      pix.red = intensity.val[2];;
      pix.green = intensity.val[1];;
      pix.blue = intensity.val[0];;
      img->set_pixel(j, i, pix);
    }
  }
}

void DepthImageCb(const sensor_msgs::ImageConstPtr& l_image_msg,
                                 const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                 const sensor_msgs::CameraInfoConstPtr& r_info_msg,
                                 const stereo_msgs::DisparityImageConstPtr& disp_msg)
{
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);
  // Calculate point cloud
  const sensor_msgs::Image& dimage = disp_msg->image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  model_.projectDisparityImageTo3d(dmat, points_mat_, true);
  cv::Mat_<cv::Vec3f> mat = points_mat_;
  //cv::imwrite("/home/ayush/point.jpg",points_mat_);
  sensor_msgs::ImagePtr depth_image(new sensor_msgs::Image());

  depth_image->header = disp_msg->header;
  depth_image->width = dimage.width;
  depth_image->height = dimage.height;
  depth_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_image->step = sizeof(float)*depth_image->width;

  size_t num_pixels = depth_image->width * depth_image->height;

  depth_image->data.resize(num_pixels * sizeof(float));

  std::vector<uint8_t>& data = depth_image->data;

  float nan = std::numeric_limits<float>::quiet_NaN();

  for (size_t i = 0; i < num_pixels; ++i){
    //data[i*sizeof(float)] =
    if (mat(i)[2] < 1000.0){
      memcpy (&data[i*sizeof(float)], &mat(i)[2], sizeof (float));
    }else{
      memcpy (&data[i*sizeof(float)], &nan, sizeof (float));//data[i*sizeof(float)]
    }
  }


  pub_depth_image.publish(depth_image);
}

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

    // tf::Vector3 tl(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    // //tf::Vector3 tl_trans(-odom.pose.pose.position.y, -odom.pose.pose.position.z, odom.pose.pose.position.x);
    tf::Vector3 position;
    
    // tf::Quaternion qt(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    // double roll, pitch, yaw; Nodes


    // tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);
    // tf::Quaternion qt_trans;
    // qt_trans.setRPY(-pitch,-yaw,roll);
    // new_pose.setOrigin(tl);
    // new_pose.setRotation(qt);

//     tf::TransformListener listener;
//     tf::StampedTransform transform;
//     try{
// listener.waitForTransform("/camera_optical", "/odom", ros::Time(0), ros::Duration(1.0) );
//       listener.lookupTransform("/camera_optical", "/odom",  
//                                ros::Time(0), transform);
//     }
//     catch (tf::TransformException ex){
//       ROS_ERROR("%s",ex.what());
//     }
//     new_pose.setOrigin(transform.getOrigin());
//     new_pose.setRotation(transform.getRotation());
    tf::Vector3 tl_odom(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    tf::Quaternion qt_odom(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    new_pose.setOrigin(tl_odom);
    new_pose.setRotation(qt_odom);

    if (!flag_1){
      old_pose = new_pose;
      ROS_INFO("old_pose init!");
      flag_1 = true;
    }
    trans = new_pose.inverseTimes(old_pose);
    tf::Vector3 tl(trans.getOrigin());
    tf::Vector3 tl_trans(-tl[1], -tl[2], tl[0]);
    tf::Quaternion qt(trans.getRotation());
    double roll, pitch, yaw;
    tf::Matrix3x3(qt).getRPY(roll, pitch, yaw);
    tf::Quaternion qt_trans;
    qt_trans.setRPY(-pitch,-yaw,roll);
    // new_pose.setOrigin(tl_trans);
    // new_pose.setRotation(qt_trans);

    tf::Matrix3x3 tf_R(qt_trans);
    tf::Vector3 tf_T = tl_trans;
    double r[3][3] = {{tf_R[0][0], tf_R[0][1], tf_R[0][2]}, {tf_R[1][0], tf_R[1][1], tf_R[1][2]}, {tf_R[2][0], tf_R[2][1], tf_R[2][2]}};//{{1,0,0},{0,1,0},{0,0,1}};//
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

    png::image<png::gray_pixel_16> segmentImage;
    png::image<png::gray_pixel_16> disparityImage;
    std::vector< std::vector<double> > disparityPlaneParameters;
    std::vector< std::vector<int> > boundaryLabels;
    png::image<png::rgb_pixel> leftImage(image_width_, image_height_);
    png::image<png::rgb_pixel> rightImage(image_width_, image_height_);
    
    //tf::Vector3 tl2 = old_pose.getOrigin();
    //ROS_INFO("Old Pos %f %f %f",tl2[0],tl2[1],tl2[2]);
    //ROS_INFO("New Pos %f %f %f",tl[0],tl[1],tl[2]);
    position = tl_trans;
    ROS_INFO("Transform %f %f %f",position[0],position[1],position[2]);
    if ((std::abs(position[0])) > 0.08f && (std::abs(position[0]) + std::abs(position[1]) + std::abs(position[2]))!= 0)
    {
      if (std::abs(position[0]) < 0.25f)
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
        ci_l->P[3] = P1.at<double>(0, 3);//-ci_l->K[0]*position[0];
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
        
        if (flag)
          {
            if(position[0] > 0)
            {
            //ROS_INFO("Reached 1"); 
            ConvertMatToPNG(cv_ptr_old->image, &rightImage);
            ConvertMatToPNG(cv_bridge::toCvShare(msg, "bgr8")->image, &leftImage);
            // leftImage.write("/home/ayush/left.png");
            // rightImage.write("/home/ayush/right.png");
            ROS_INFO("Reached 2");

            sps.compute(superpixelTotal, leftImage, rightImage, segmentImage, disparityImage, disparityPlaneParameters, boundaryLabels);
            cv::Mat cv_disparity;
            ROS_INFO("Reached 3");
            ConvertDispImageToCvMat8(disparityImage, &cv_disparity);
            
            disp_msg->T                     = position[0];
            disp_msg->f                     = ci_l->K[0];
            disp_msg->delta_d               = 1.0 / 16;
            disp_msg->header.stamp          = ros::Time::now();
            disp_msg->header.frame_id       = "camera_optical"; 
            //disp_msg->header.seq            = count++; //a counter, int type
            sensor_msgs::Image& dimage = disp_msg->image;
            dimage.width  = cv_disparity.size().width ;
            dimage.height = cv_disparity.size().height ;
            dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            dimage.step = dimage.width * sizeof(float);
            dimage.data.resize(dimage.step * dimage.height);
            cv::Mat_<float> dmat(dimage.height, dimage.width, reinterpret_cast<float*>(&dimage.data[0]), dimage.step);
            cv_disparity.convertTo(dmat,dmat.type());
            DepthImageCb(msg, ci_l, ci_r, disp_msg);
            image_pub_2.publish(msg, ci_l);
            image_pub_1.publish(cv_ptr_old->toImageMsg(), ci_r);
          }
          else
          {
            ConvertMatToPNG(cv_ptr_old->image, &leftImage);
            ConvertMatToPNG(cv_bridge::toCvShare(msg, "bgr8")->image, &rightImage);
            // leftImage.write("/home/ayush/left.png");
            // rightImage.write("/home/ayush/right.png");
            ROS_INFO("Reached 2");

            sps.compute(superpixelTotal, leftImage, rightImage, segmentImage, disparityImage, disparityPlaneParameters, boundaryLabels);
            cv::Mat cv_disparity;
            ROS_INFO("Reached 3");
            ConvertDispImageToCvMat8(disparityImage, &cv_disparity);
            cv::imwrite("/home/ayush/disp.png",cv_disparity);
            disp_msg->T                     = position[0];
            disp_msg->f                     = ci_l->K[0];
            disp_msg->delta_d               = 1.0 / 16;
            disp_msg->header.stamp          = ros::Time::now();
            disp_msg->header.frame_id       = "camera_optical"; 
            //disp_msg->header.seq            = count++; //a counter, int type
            sensor_msgs::Image& dimage = disp_msg->image;
            dimage.width  = cv_disparity.size().width ;
            dimage.height = cv_disparity.size().height ;
            dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            dimage.step = dimage.width * sizeof(float);
            dimage.data.resize(dimage.step * dimage.height);
            cv::Mat_<float> dmat(dimage.height, dimage.width, reinterpret_cast<float*>(&dimage.data[0]), dimage.step);
            cv_disparity.convertTo(dmat,dmat.type());
            DepthImageCb(cv_ptr_old->toImageMsg(), ci_l, ci_r, disp_msg);
            image_pub_2.publish(cv_ptr_old->toImageMsg(), ci_l);
            image_pub_1.publish(msg, ci_r);
          }
            //cv::waitKey(30);
            //ROS_INFO("Reached 5");
          }
        old_pose = new_pose;
        //ROS_INFO("Old pose set");
        cv_ptr_old = cv_bridge::toCvCopy(msg, "bgr8");
        flag = true;
      }
      else
      {
        old_pose = new_pose;
        //ROS_INFO("Old pose set");
        cv_ptr_old = cv_bridge::toCvCopy(msg, "bgr8");
        flag = true;
      }
    }

    

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
   // cv::namedWindow("view");
   // cv::startWindowThread();
  image_transport::ImageTransport it(nh);


  //nh.param("camera_frame_id", img_.header.frame_id, std::string("stereo"));
  nh.param("image_topic",image_topic_,std::string("/camera/image_raw"));
  nh.param("odom_topic",odom_topic_,std::string("/bebop/odom"));
  nh.param("camera_name", camera_name_, std::string("camera"));
  nh.param("camera_info_url", camera_info_url_, std::string(""));
  nh.param("image_width", image_width_, 640);
  nh.param("image_height", image_height_, 368);
  ROS_INFO("camera: %s",camera_info_url_.c_str());
  
  image_transport::Subscriber sub = it.subscribe(image_topic_, 1, imageCallback);
  image_pub_1 = it.advertiseCamera("/camera/left/image_raw", 1);
  image_pub_2 = it.advertiseCamera("/camera/right/image_raw", 1);
  pub_depth_image  = nh.advertise<sensor_msgs::Image>("/camera/depth_image", 1);
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

  
  sps.setIterationTotal(outerIterationTotal, innerIterationTotal);
  sps.setWeightParameter(lambda_pos, lambda_depth, lambda_bou, lambda_smo);
  sps.setInlierThreshold(lambda_d);
  sps.setPenaltyParameter(lambda_hinge, lambda_occ, lambda_pen);

  disp_msg->min_disparity = 0;
  disp_msg->min_disparity = 32;

  // should be safe
  disp_msg->valid_window.x_offset = 0;
  disp_msg->valid_window.y_offset = 0;
  disp_msg->valid_window.width    = 0;
  disp_msg->valid_window.height   = 0;

  ros::spin();
  // cv::destroyWindow("view");
}
