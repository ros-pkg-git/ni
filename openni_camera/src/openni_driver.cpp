/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010
 *    Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *    William Morris <morris@ee.ccny.cuny.edu>
 *    Stéphane Magnenat <stephane.magnenat@mavt.ethz.ch>
 *    Radu Bogdan Rusu <rusu@willowgarage.com>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "openni_camera/openni.h"
#include <sensor_msgs/image_encodings.h>
#include <boost/make_shared.hpp>

namespace openni_camera 
{

typedef union
{
	 struct /*anonymous*/
   {
    unsigned char Blue; // Blue channel
    unsigned char Green; // Green channel
    unsigned char Red; // Red channel
    unsigned char Alpha; // alpha
   };
   float float_value;
   long long_value;
} RGBValue;

const double OpenNIDriver::SHIFT_SCALE = 0.125;

/** \brief Constructor */
OpenNIDriver::OpenNIDriver (ros::NodeHandle comm_nh, ros::NodeHandle param_nh)
  : comm_nh_ (comm_nh),
    reconfigure_server_(param_nh),
    width_ (640), height_ (480),
    started_(false), shadow_value_ (0), no_sample_value_ (0)
{
  // Init the OpenNI context
  rc_ = context_.Init ();

  if (rc_ != XN_STATUS_OK)
  {
    ROS_ERROR ("[OpenNIDriver] Init: %s", xnGetStatusString (rc_));
    return;
  }
  ROS_INFO ("[OpenNIDriver] Initialization successful.");

  // Set up reconfigure server
  ReconfigureServer::CallbackType f = boost::bind(&OpenNIDriver::configCb, this, _1, _2);
  reconfigure_server_.setCallback(f);
  
  // Assemble the point cloud data
  std::string openni_depth_frame;
  param_nh.param ("openni_depth_frame", openni_depth_frame, std::string ("/openni_depth"));
  cloud_.header.frame_id = cloud2_.header.frame_id = openni_depth_frame;
  cloud_.channels.resize (1);
  cloud_.channels[0].name = "rgb";
  cloud_.channels[0].values.resize (width_ * height_);
  /// @todo "u" and "v" channels?

  cloud2_.height = height_;
  cloud2_.width = width_;
  cloud2_.fields.resize (4);
  cloud2_.fields[0].name = "x";
  cloud2_.fields[1].name = "y";
  cloud2_.fields[2].name = "z";
  cloud2_.fields[3].name = "rgb";

  // Set all the fields types accordingly
  int offset = 0;
  for (size_t s = 0; s < cloud2_.fields.size (); ++s, offset += 4)
  {
    cloud2_.fields[s].offset   = offset;
    cloud2_.fields[s].count    = 1;
    cloud2_.fields[s].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud2_.point_step = offset;
  cloud2_.row_step   = cloud2_.point_step * cloud2_.width;
  cloud2_.data.resize (cloud2_.row_step   * cloud2_.height);
  cloud2_.is_dense = true;

  // Assemble the depth image data
  depth_image_.header.frame_id = openni_depth_frame;
  depth_image_.height = height_;
  depth_image_.width = width_;
  depth_image_.encoding = "32FC1";
  depth_image_.step = width_ * sizeof (float);
  depth_image_.data.resize (depth_image_.step * depth_image_.height);

  // Assemble the image data
  std::string openni_RGB_frame;
  param_nh.param ("openni_rgb_frame", openni_RGB_frame, std::string ("/openni_rgb"));
  rgb_image_.header.frame_id = openni_RGB_frame;
  rgb_image_.height = height_;
  rgb_image_.width = width_;
  rgb_info_.header.frame_id = rgb_image_.header.frame_id; 

  // Read calibration parameters from disk
  std::string cam_name, rgb_info_url, depth_info_url;
  param_nh.param ("camera_name", cam_name, std::string("camera"));
  param_nh.param ("rgb/camera_info_url", rgb_info_url, std::string("auto"));
  param_nh.param ("depth/camera_info_url", depth_info_url, std::string("auto"));
  if (rgb_info_url.compare("auto") == 0) 
    rgb_info_url = std::string("file://")+ros::package::getPath(ROS_PACKAGE_NAME)+std::string("/info/calibration_rgb.yaml");
  if (depth_info_url.compare("auto") == 0)
    depth_info_url = std::string("file://")+ros::package::getPath(ROS_PACKAGE_NAME)+std::string("/info/calibration_depth.yaml");
  ROS_INFO ("[OpenNIDriver] Calibration URLs:\n\tRGB: %s\n\tDepth: %s",
            rgb_info_url.c_str (), depth_info_url.c_str ());

  rgb_info_manager_   = boost::make_shared<CameraInfoManager> (ros::NodeHandle(comm_nh, "rgb"),
                                                               cam_name, rgb_info_url);
  depth_info_manager_ = boost::make_shared<CameraInfoManager> (ros::NodeHandle(comm_nh, "depth"),
                                                               cam_name, depth_info_url);
  rgb_info_   = rgb_info_manager_->getCameraInfo ();
  depth_info_ = depth_info_manager_->getCameraInfo ();
  rgb_info_.header.frame_id = openni_RGB_frame; 
  depth_info_.header.frame_id = openni_depth_frame;
  rgb_model_.fromCameraInfo(rgb_info_);
  depth_model_.fromCameraInfo(depth_info_);

  /// @todo Distinguish calibrated/uncalibrated opennis
  // Read additional calibration parameters
  param_nh.param ("shift_offset", shift_offset_, 1084.0);
  param_nh.param ("projector_depth_baseline", baseline_, 0.075); // 7.5cm

  // Compute transform matrix from (u,v,d) of depth camera to (u,v) of RGB camera
  // From (u,v,d,1) in depth image to (X,Y,Z,W) in depth camera frame
  Eigen::Matrix4d Q;
  Q << 1, 0, 0, -depth_model_.cx(),
       0, 1, 0, -depth_model_.cy(),
       0, 0, 0,  depth_model_.fx(),
       0, 0, 1.0 / baseline_, 0;

  // From (X,Y,Z,W) in depth camera frame to RGB camera frame
  Eigen::Matrix4d S;
  XmlRpc::XmlRpcValue rot, trans;
  if (param_nh.getParam("depth_rgb_rotation", rot) &&
      param_nh.getParam("depth_rgb_translation", trans) &&
      rot.size() == 9 && trans.size() == 3)
  {
    S << rot[0], rot[1], rot[2], trans[0],
         rot[3], rot[4], rot[5], trans[1],
         rot[6], rot[7], rot[8], trans[2],
         0,      0,      0,      1;
  }
  else
  {
    ROS_WARN("Transform between depth and RGB cameras is not calibrated");
    S << 1, 0, 0, -0.025, // 2.5cm
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
  }

  // From (X,Y,Z,W) in RGB camera frame to (u,v,w) in RGB image
  Eigen::Matrix<double, 3, 4> P;
  P << rgb_model_.fx(), 0,               rgb_model_.cx(), 0,
       0,               rgb_model_.fy(), rgb_model_.cy(), 0,
       0,               0,               1,               0;

  // Putting it all together, from (u,v,d,1) in depth image to (u,v,w) in RGB image
  depth_to_rgb_ = P*S*Q;

  std::cout << "Transform matrix:" << std::endl << depth_to_rgb_ << std::endl << std::endl;

  // Publishers and subscribers
  image_transport::ImageTransport it(comm_nh);
  pub_rgb_     = it.advertiseCamera ("rgb/image_raw", 1);
  pub_rgb_rect_ = it.advertise("rgb/image_rect_color", 1);
  pub_depth_   = it.advertiseCamera ("depth/image_raw", 1);
  pub_ir_      = it.advertiseCamera ("ir/image_raw", 1);
  //pub_depth_points_  = comm_nh.advertise<sensor_msgs::PointCloud> ("depth/points", 15);
  pub_depth_points2_ = comm_nh.advertise<sensor_msgs::PointCloud2>("depth/points2", 15);
  //pub_rgb_points2_   = comm_nh.advertise<sensor_msgs::PointCloud2>("rgb/points2", 15);
  pub_imu_ = comm_nh.advertise<sensor_msgs::Imu>("imu", 15);
}

/** \brief Initialize a openni device, given an index.
  * \param index the index of the device to initialize
  */
bool
  OpenNIDriver::init (int index)
{
// TODO: switch between several devices based on the index
  // Create a DepthGenerator node
  rc_ = depth_.Create (context_);
  if (rc_ != XN_STATUS_OK)
  {
    ROS_ERROR ("[OpenNIDriver] %s", xnGetStatusString (rc_));
    return (false);
  }
  rc_ = image_.Create (context_);
  if (rc_ != XN_STATUS_OK)
  {
    ROS_ERROR ("[OpenNIDriver] %s", xnGetStatusString (rc_));
    return (false);
  }

  // Set the correct mode on the depth/image generator
  XnMapOutputMode mode;
  mode.nXRes = width_;
  mode.nYRes = height_;
  mode.nFPS  = 30;
  rc_ = depth_.SetMapOutputMode (mode);
  rc_ = image_.SetMapOutputMode (mode);

  depth_.GetMetaData (depth_md_);
  image_.GetMetaData (image_md_);

  // Read parameters from the camera
  if (depth_.GetRealProperty ("ZPPS", pixel_size_) != XN_STATUS_OK)
    ROS_ERROR ("[OpenNIDriver] Could not read pixel size!");
  else
    ROS_INFO_STREAM ("[OpenNIDriver] Pixel size: " << pixel_size_);

  pixel_size_ *= 1280 / width_;

  if (depth_.GetIntProperty ("ZPD", F_) != XN_STATUS_OK)
    ROS_ERROR ("[OpenNIDriver] Could not read virtual plane distance!");
  else
    ROS_INFO_STREAM ("[OpenNIDriver] Virtual plane distance: " << F_);

  if (depth_.GetRealProperty ("LDDIS", baseline_) != XN_STATUS_OK)
    ROS_ERROR ("[OpenNIDriver] Could not read base line!");
  else
    ROS_INFO_STREAM ("[OpenNIDriver] Base line: " << baseline_);

  focal_length_ = (double)F_/pixel_size_;

  if (depth_.GetIntProperty ("ShadowValue", shadow_value_) != XN_STATUS_OK)
    ROS_WARN ("[OpenNIDriver] Could not read shadow value!");

  if (depth_.GetIntProperty ("NoSampleValue", no_sample_value_) != XN_STATUS_OK)
    ROS_WARN ("[OpenNIDriver] Could not read no sample value!");

  if (image_.SetIntProperty ("InputFormat", 6) != XN_STATUS_OK)
    ROS_ERROR ("[OpenNIDriver] Error setting the RGB output format to Uncompressed 8-bit BAYER!");

  XnUInt64 fps;
  depth_.GetIntProperty ("FPS", fps);
  ROS_INFO_STREAM ("[OpenNIDriver] FPS: " << fps);

	depth_.GetAlternativeViewPointCap().SetViewPoint( image_ );
  return (true);
}

bool OpenNIDriver::spin ()
{
  if (!started_)
    return (false);

  while (comm_nh_.ok ())
  {
    // Wait for new data to be available
    rc_ = context_.WaitOneUpdateAll (depth_);
    if (rc_ != XN_STATUS_OK)
    {
      ROS_ERROR ("[OpenNIDriver::spin] Error receiving data: %s", xnGetStatusString (rc_));
      continue;
    }

    // Take current RGB and depth map
    depth_buf_ = depth_.GetDepthMap ();
    rgb_buf_   = image_.GetImageMap ();
    // And publish them
    publish ();

    // Spin for dynamic reconfigure
    ros::spinOnce ();
  }
  return (true);
}

/** \brief Destructor */
OpenNIDriver::~OpenNIDriver ()
{
  stop ();
  context_.Shutdown ();
}

/** \brief Start (resume) the data acquisition process. */
bool
  OpenNIDriver::start ()
{
  started_ = false;
  // Make OpenNI start generating data
  rc_ = context_.StartGeneratingAll ();

  if (rc_ != XN_STATUS_OK)
  {
    ROS_ERROR ("[OpenNIDriver::start] Error in start (): %s", xnGetStatusString (rc_));
    return (false);
  }

  // Turn on frame synchronization
  if (depth_.SetIntProperty ("FrameSync", 0) != XN_STATUS_OK)
    ROS_ERROR ("[OpenNIDriver] Could not turn on frame synchronization!");

  started_ = true;
  return (true);
}

/** \brief Stop (pause) the data acquisition process. */
void 
  OpenNIDriver::stop ()
{
  context_.StopGeneratingAll ();
  started_ = false;
}

void OpenNIDriver::publish ()
{
  ros::Time time = ros::Time::now ();
  cloud_.header.stamp = cloud2_.header.stamp = time;
  rgb_image_.header.stamp   = rgb_info_.header.stamp   = time;
  depth_image_.header.stamp = depth_info_.header.stamp = time;

  // Fill raw RGB image message
  if (pub_rgb_.getNumSubscribers () > 0)
  {
    // Copy the image data
    memcpy (&rgb_image_.data[0], &rgb_buf_[0], rgb_image_.data.size());
    pub_rgb_.publish (boost::make_shared<const sensor_msgs::Image> (rgb_image_), 
                      boost::make_shared<const sensor_msgs::CameraInfo> (rgb_info_)); 
  }
/*
  // Rectify the RGB image if necessary
  cv::Mat rgb_raw(height_, width_, CV_8UC3, rgb_buf_);
  cv::Mat rgb_rect;
  if (pub_depth_points_.getNumSubscribers () > 0 ||
      pub_depth_points2_.getNumSubscribers () > 0)
    rgb_model_.rectifyImage(rgb_raw, rgb_rect);
  double fT = depth_model_.fx() * baseline_;
  }
*/  
  if (pub_depth_points2_.getNumSubscribers () > 0)
  {
    // Assemble an awesome sensor_msgs/PointCloud2 message
    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    int k = 0;
    
    float* pt_data = reinterpret_cast<float*>(&cloud2_.data[0] );
    for (int v = 0; v < height_; ++v)
    {
      for (int u = 0; u < width_; ++u, ++k, pt_data += 4 /*cloud2_.step*/) 
      {
        //float* pt_data = reinterpret_cast<float*>(&cloud2_.data[0] + k * cloud2_.point_step);

        if (depth_md_[k] == 0 || depth_md_[k] == no_sample_value_ || depth_md_[k] == shadow_value_)
        {
          // not valid
          pt_data[0] = bad_point;
          pt_data[1] = bad_point;
          pt_data[2] = bad_point;
          // Fill in RGB
          pt_data[3] = 0;
          continue;
        }

        // Fill in XYZ
        pt_data[0] = (u - 320) * pixel_size_ * depth_md_[k] * 0.001 / F_ ;
        pt_data[1] = (v - 240) * pixel_size_ * depth_md_[k] * 0.001 / F_ ;
        pt_data[2] = depth_md_[k] * 0.001;

        // Fill in coolor
        RGBValue color;
        color.Red   = rgb_buf_[ k * 3 ];
        color.Green = rgb_buf_[ k * 3 + 1];
        color.Blue  = rgb_buf_[ k * 3 + 2];
        color.Alpha = 0;
        pt_data[3] = color.float_value;
      }
    }
    pub_depth_points2_.publish (boost::make_shared<const sensor_msgs::PointCloud2> (cloud2_));
  }
 
  if (pub_depth_.getNumSubscribers () > 0)
  { 
    // Fill in the depth image data
    // iterate over all elements and fill disparity matrix: disp[x,y] = f * b / z_distance[x,y];
    float* pixel = reinterpret_cast<float*>(&depth_image_.data[0]);
    for (register int i = 0; i < width_ * height_; ++i, ++pixel)
    {
      if (depth_md_[i] == 0 || depth_md_[i] == no_sample_value_ || depth_md_[i] == shadow_value_)
        *pixel = 0.0;
      else
        *pixel = focal_length_ * baseline_ * 1000.0 / (double)depth_md_[i];
    }

    // Publish depth Image
    pub_depth_.publish (boost::make_shared<const sensor_msgs::Image> (depth_image_), 
                        boost::make_shared<const sensor_msgs::CameraInfo> (depth_info_));
  }
  // Publish RGB or IR Image
/*  if (config_.color_format == FORMAT_IR)
  {
    if (pub_ir_.getNumSubscribers() > 0)
      pub_ir_.publish (boost::make_shared<const sensor_msgs::Image> (rgb_image_), boost::make_shared<const sensor_msgs::CameraInfo> (depth_info_));
  }
  else */
}


void OpenNIDriver::publishImu()
{
  imu_msg_.header.stamp = ros::Time::now();
  imu_msg_.linear_acceleration.x = accel_x_;
  imu_msg_.linear_acceleration.y = accel_y_;
  imu_msg_.linear_acceleration.z = accel_z_;
  imu_msg_.linear_acceleration_covariance[0] = imu_msg_.linear_acceleration_covariance[4]
      = imu_msg_.linear_acceleration_covariance[8] = 0.01; // @todo - what should these be?
  imu_msg_.angular_velocity_covariance[0] = -1; // indicates angular velocity not provided
  imu_msg_.orientation_covariance[0] = -1; // indicates orientation not provided
  if (pub_imu_.getNumSubscribers() > 0)
    pub_imu_.publish(imu_msg_);
}

void OpenNIDriver::configCb (Config &config, uint32_t level)
{
    rgb_image_.encoding = sensor_msgs::image_encodings::RGB8;
    rgb_image_.data.resize (width_ * height_ * 4);
    rgb_image_.step = width_ * 3;
  /// @todo Integrate init() in here, so can change device and not worry about first config call
  
  // Configure color output to be RGB or Bayer
  /// @todo Mucking with image_ here might not be thread-safe
/*  if (config.color_format == FORMAT_RGB) {
    rgb_image_.encoding = sensor_msgs::image_encodings::RGB8;
    rgb_image_.data.resize (width_ * height_ * 4);
    rgb_image_.step = width_ * 3;
  }
  else if (config.color_format == FORMAT_IR) {
    rgb_image_.encoding = sensor_msgs::image_encodings::MONO8;
    rgb_image_.data.resize (width_ * height_ * 4);
    rgb_image_.step = width_ * 3;
  }
  else {
    ROS_ERROR("Unknown color format code %d", config.color_format);
  }*/

  config_ = config;
  updateDeviceSettings();
}

void OpenNIDriver::updateDeviceSettings()
{
}

} // namespace openni_camera
