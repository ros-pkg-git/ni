/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010
 *    Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *    William Morris <morris@ee.ccny.cuny.edu>
 *    Stéphane Magnenat <stephane.magnenat@mavt.ethz.ch>
 *    Radu Bogdan Rusu <rusu@willowgarage.com>
 *    Suat Gedikli <gedikli@willowgarage.com>
 *    Patrick Mihelich <mihelich@willowgarage.com>
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
#include <sensor_msgs/fill_image.h>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <boost/make_shared.hpp>

// Branch on whether we have the changes to CameraInfo in unstable
#if ROS_VERSION_MINIMUM(1, 3, 0)
#include <sensor_msgs/distortion_models.h>
#endif

using namespace std;

namespace openni_camera 
{
// Suat: This magic number is the focal length of the RGB camera and comes from our Primesense/Kinect calibration routine.
const double OpenNIDriver::rgb_focal_length_VGA_ = 525;
const string OpenNIDriver::rgb_frame_id_ = "openni_rgb_optical_frame";
const string OpenNIDriver::IR_frame_id_ = "openni_depth_optical_frame";

typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue; // Blue channel
    unsigned char Green; // Green channel
    unsigned char Red; // Red channel
    unsigned char Alpha; // Alpha channel
  };
  float float_value;
  long long_value;
} RGBValue;


/** \brief Constructor */
OpenNIDriver::OpenNIDriver (ros::NodeHandle comm_nh, ros::NodeHandle param_nh)
  : comm_nh_ (comm_nh)
  , param_nh_ (param_nh)
  , reconfigure_server_ (param_nh)
  , shadow_value_ (0)
  , no_sample_value_ (0)
{
  // Init the OpenNI context
  XnStatus status = context_.Init ();
  if (status != XN_STATUS_OK)
  {
    ROS_ERROR ("[OpenNIDriver] Init: %s", xnGetStatusString (status));
    return;
  }


  // Set up reconfigure server
  ReconfigureServer::CallbackType f = boost::bind(&OpenNIDriver::configCb, this, _1, _2);
  reconfigure_server_.setCallback(f);
  
  // set frame id of images that do not change their coordinate system!
  // other than unregistered images, clouds etc registered are in the rgb_frame
  rgb_image_.header.frame_id  = rgb_frame_id_;
  gray_image_.header.frame_id = rgb_frame_id_;
  rgb_info_.header.frame_id   = rgb_frame_id_;

  // Publishers and subscribers
  image_transport::ImageTransport it(comm_nh);
  //pub_bayer_   = it.advertise ("rgb/image_raw", 15);
  pub_rgb_     = it.advertiseCamera ("rgb/image_color", 15); /// @todo 15 looks like overkill
  pub_gray_    = it.advertiseCamera ("rgb/image_mono", 15);
  pub_depth_image_ = it.advertiseCamera ("depth/image", 15);
  pub_disparity_ = comm_nh_.advertise<stereo_msgs::DisparityImage>("depth/disparity", 15 );
  pub_depth_points2_ = comm_nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("depth/points2", 15);

  SyncPolicy sync_policy(4); // queue size
  /// @todo Set inter-message lower bound, age penalty, max interval to lower latency
  // Connect no inputs, we'll add messages manually
  depth_rgb_sync_.reset( new Synchronizer(sync_policy) );
  depth_rgb_sync_->registerCallback(boost::bind(&OpenNIDriver::publishXYZRGBPointCloud, this, _1, _2));
}

/** \brief Destructor */
OpenNIDriver::~OpenNIDriver ()
{
  context_.StopGeneratingAll ();
  context_.Shutdown ();
}

/** \brief Spin loop. */
bool OpenNIDriver::spin ()
{
  XnStatus status;
  ros::Duration r (0.01);

  while (comm_nh_.ok ())
  {
    // Spin for ROS message processing
    ros::spinOnce (); // At top to allow use of continue below. 
    
    if (!isDepthStreamRequired() && depth_generator_.IsGenerating())
    {
      status = depth_generator_.StopGenerating();
      if (status != XN_STATUS_OK)
      {
        ROS_ERROR ("[OpenNIDriver::spin] Error in stopping depth stream (): %s", xnGetStatusString (status));
        return (false);
      }
    }
    else if (isDepthStreamRequired() && !depth_generator_.IsGenerating())
    {
      status = depth_generator_.StartGenerating();
      if (status != XN_STATUS_OK)
      {
        ROS_ERROR ("[OpenNIDriver::spin] Error in starting depth stream (): %s", xnGetStatusString (status));
        return (false);
      }
      // Need to update the settings everytime the stream is started
      updateDeviceSettings ();
    }

    if (!isImageStreamRequired() && image_generator_.IsGenerating())
    {
      status = image_generator_.StopGenerating();
      if (status != XN_STATUS_OK)
      {
        ROS_ERROR ("[OpenNIDriver::spin] Error in stopping image stream (): %s", xnGetStatusString (status));
        return (false);
      }
    }
    else if (isImageStreamRequired() && !image_generator_.IsGenerating())
    {
      status = image_generator_.StartGenerating();
      if (status != XN_STATUS_OK)
      {
        ROS_ERROR ("[OpenNIDriver::spin] Error in starting image stream (): %s", xnGetStatusString (status));
        return (false);
      }
      // Need to update the settings everytime the stream is started
      updateDeviceSettings ();
    }

    if (!isImageStreamRequired() && !isDepthStreamRequired())
    {
      r.sleep();
      continue;
    }

    // See if new data is available
    if (depth_generator_.IsNewDataAvailable())
    {
      /// @todo Maybe take ROS time here and have more accurate offset
      depth_generator_.WaitAndUpdateData(); // non-blocking
      processDepth();
    }
    if (image_generator_.IsNewDataAvailable())
    {
      image_generator_.WaitAndUpdateData(); // non-blocking
      processRgb();
    }

    r.sleep(); /// @todo should only happen if no new data
  }
  return (true);
}

void OpenNIDriver::processDepth ()
{
  /// @todo Some sort of offset based on the hardware time
  ros::Time time = ros::Time::now();
  
  xn::DepthMetaData depth_md;
  depth_generator_.GetMetaData( depth_md );

  // Raw depth image
  if (pub_depth_image_.getNumSubscribers () > 0)
    publishDepthImage ( depth_md, time );

  // Disparity image
  if (pub_disparity_.getNumSubscribers () > 0)
  {
    publishDisparity ( depth_md, time );
  }

  // Point cloud
  if (pub_depth_points2_.getNumSubscribers() > 0 )
  {
    if (config_.point_cloud_type == OpenNI_XYZRGB)
    {
      sensor_msgs::ImagePtr depth_ptr = boost::make_shared<sensor_msgs::Image>();
      depth_ptr->header.stamp = time;
      depth_ptr->header.frame_id = disp_image_.header.frame_id;
      depth_ptr->header.seq = depth_md.FrameID();
      sensor_msgs::fillImage(*depth_ptr, sensor_msgs::image_encodings::TYPE_16UC1,
                             depth_md.YRes(), depth_md.XRes(), depth_md.XRes() * sizeof(uint16_t),
                             (void*)depth_md.Data());
      depth_rgb_sync_->add<0>(depth_ptr);
    }
    else
      publishXYZPointCloud(depth_md, time);
  }
}

void OpenNIDriver::processRgb ()
{
  /// @todo Some sort of offset based on the hardware time
  ros::Time time = ros::Time::now();
  
  xn::ImageMetaData image_md;
  image_generator_.GetMetaData( image_md );
  /*
  if (pub_bayer_.getNumSubscribers() > 0)
  {
    if (config_.device_type == OpenNI_Kinect)
    {
      sensor_msgs::ImagePtr bayer_ptr = boost::make_shared<sensor_msgs::Image>();
      bayer_ptr->header.stamp = time;
      bayer_ptr->header.seq   = image_md.FrameID();
      bayer_ptr->header.frame_id = rgb_frame_id_;
      sensor_msgs::fillImage(*bayer_ptr, sensor_msgs::image_encodings::BAYER_GRBG8,
                             image_md.YRes(), image_md.XRes(), image_md.XRes(),
                             (void*)image_md.Data());
      pub_bayer_.publish(bayer_ptr);
    }
    else
    {
      ROS_WARN("[OpenNIDriver] : no bayer image available for primesense device" );
    }
  }
*/
  if (isRGBRequired())
  {
    rgb_image_.header.stamp = rgb_info_.header.stamp = time;
    rgb_image_.header.seq   = rgb_info_.header.seq   = image_md.FrameID();

    if( config_.device_type == OpenNI_Primesense )
      YUV2RGB( image_md, rgb_image_ );
    else if( config_.device_type == OpenNI_Kinect )
      bayer2RGB( image_md, rgb_image_, config_.Debayering );
    else
      ROS_ERROR("[OpenNIDriver::processRgb] unknown device type");

    sensor_msgs::ImageConstPtr rgb_ptr = boost::make_shared<const sensor_msgs::Image> (rgb_image_);
    if (pub_rgb_.getNumSubscribers() > 0)
    {
      pub_rgb_.publish (rgb_ptr, boost::make_shared<const sensor_msgs::CameraInfo> (rgb_info_));
    }

    if (pub_depth_points2_.getNumSubscribers() > 0 && config_.point_cloud_type == OpenNI_XYZRGB)
      depth_rgb_sync_->add<1>(rgb_ptr);
  }

  if (isGrayRequired())
  {
    gray_image_.header.stamp = rgb_info_.header.stamp = time;
    gray_image_.header.seq   = rgb_info_.header.seq   = image_md.FrameID ();

    if( config_.device_type == OpenNI_Primesense )
      YUV2Gray( image_md, gray_image_ );
    else if( config_.device_type == OpenNI_Kinect )
      bayer2Gray( image_md, gray_image_, config_.Debayering );
    else
      ROS_ERROR("[OpenNIDriver::processRgb] unknown device type");
    
    if (pub_gray_.getNumSubscribers() > 0)
    {
      pub_gray_.publish (boost::make_shared<const sensor_msgs::Image> (gray_image_),
                         boost::make_shared<const sensor_msgs::CameraInfo> (rgb_info_));
    }
  }
}

void OpenNIDriver::publishDepthImage ( const xn::DepthMetaData& depth_md, ros::Time time )
{
  /// @todo Disentangle this from disparity image values
  sensor_msgs::ImagePtr msg_ptr = boost::make_shared<sensor_msgs::Image> ();
  msg_ptr->header.stamp = time;
  msg_ptr->header.frame_id = disp_image_.header.frame_id; /// @todo Depends on registration
  msg_ptr->header.seq = depth_md.FrameID();
  msg_ptr->height = disp_image_.image.height;
  msg_ptr->width  = disp_image_.image.width;
  msg_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  msg_ptr->step = msg_ptr->width * sizeof(float);
  msg_ptr->data.resize(msg_ptr->height * msg_ptr->step);
  
  unsigned xStep = depth_stream_width / msg_ptr->width;
  unsigned ySkip = (depth_stream_height / msg_ptr->height - 1) * depth_stream_width;

  // Fill in the depth image data, converting mm to m
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  float* pixel = reinterpret_cast<float*>(&msg_ptr->data[0]);
  unsigned depthIdx = 0;

  for (unsigned yIdx = 0; yIdx < msg_ptr->height; ++yIdx, depthIdx += ySkip )
  {
    for (unsigned xIdx = 0; xIdx < msg_ptr->width; ++xIdx, depthIdx += xStep, ++pixel)
    {
      /// @todo Different values for these cases
      if (depth_md[depthIdx] == 0 ||
          depth_md[depthIdx] == no_sample_value_ ||
          depth_md[depthIdx] == shadow_value_)
        *pixel = bad_point;
      else
        *pixel = (float)depth_md[depthIdx] * 0.001f;
    }
  }

  depth_info_.header.seq = depth_md.FrameID();
  depth_info_.header.stamp = time;
  pub_depth_image_.publish (msg_ptr, boost::make_shared<const sensor_msgs::CameraInfo> (depth_info_));
}

void OpenNIDriver::publishDisparity ( const xn::DepthMetaData& depth_md, ros::Time time )
{
  unsigned xStep = depth_stream_width / disp_image_.image.width;
  unsigned ySkip = (depth_stream_height / disp_image_.image.height - 1) * depth_stream_width;

  // Fill in the depth image data
  // iterate over all elements and fill disparity matrix: disp[x,y] = f * b / z_distance[x,y];
  float constant;
  if( config_.point_cloud_type == OpenNI_XYZ_unregistered)
    constant = depth_focal_length_VGA_ * baseline_ * 1000.0;
  else
    constant = rgb_focal_length_VGA_ * baseline_ * 1000.0;

  float* pixel = reinterpret_cast<float*>(&disp_image_.image.data[0]);
  unsigned depthIdx = 0;

  for (unsigned yIdx = 0; yIdx < disp_image_.image.height; ++yIdx, depthIdx += ySkip )
  {
    for (unsigned xIdx = 0; xIdx < disp_image_.image.width; ++xIdx, depthIdx += xStep, ++pixel)
    {
      if (depth_md[depthIdx] == 0 ||
          depth_md[depthIdx] == no_sample_value_ ||
          depth_md[depthIdx] == shadow_value_)
        *pixel = 0.0;
      else
        *pixel = constant / (double)depth_md[depthIdx];
    }
  }
  
  // Publish disparity Image
  disp_image_.header.stamp = time;
  disp_image_.header.seq = depth_md.FrameID();
  pub_disparity_.publish ( disp_image_ );
}

void 
OpenNIDriver::publishXYZPointCloud (const xn::DepthMetaData& depth_md, ros::Time time) const
{
  pcl::PointCloud<pcl::PointXYZ> cloud_out;
  cloud_out.header = cloud2_.header;
  cloud_out.height = cloud2_.height;
  cloud_out.width = cloud2_.width;
  cloud_out.is_dense = cloud2_.is_dense;

  cloud_out.points.resize(cloud_out.height * cloud_out.width);

  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  int depth_idx = 0;
  
  float constant;
  if (config_.point_cloud_type == OpenNI_XYZ_unregistered)
    constant = 0.001 / depth_focal_length_VGA_;
  else
    constant = 0.001 / rgb_focal_length_VGA_;

  unsigned depth_step = depth_md.XRes () / cloud_out.width;
  unsigned depth_skip = (depth_md.YRes () / cloud_out.height - 1) * depth_md.XRes ();

  int centerX = cloud_out.width >> 1;
  int centerY = cloud_out.height >> 1;
  constant *= depth_step;

  for (int v = 0; v < (int)cloud_out.height; ++v, depth_idx += depth_skip)
  {
    for (int u = 0; u < (int)cloud_out.width; ++u, depth_idx += depth_step)
    {
      pcl::PointXYZ& pt = cloud_out(u, v);
      
      // Check for invalid measurements
      if (depth_md[depth_idx] == 0 ||
          depth_md[depth_idx] == no_sample_value_ ||
          depth_md[depth_idx] == shadow_value_)
      {
        // not valid
        pt.x = bad_point;
        pt.y = bad_point;
        pt.z = bad_point;
        continue;
      }

      // Fill in XYZ
      pt.x = (u - centerX) * depth_md[depth_idx] * constant;
      pt.y = (v - centerY) * depth_md[depth_idx] * constant;
      pt.z = depth_md[depth_idx] * 0.001;
    }
  }

  cloud_out.header.frame_id = disp_image_.header.frame_id;
  cloud_out.header.seq = depth_md.FrameID ();
  cloud_out.header.stamp = time;

  pub_depth_points2_.publish (cloud_out.makeShared ());
}

void 
OpenNIDriver::publishXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg,
                                       const sensor_msgs::ImageConstPtr& rgb_msg) const
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
  cloud_out.header = cloud2_.header;
  cloud_out.height = cloud2_.height;
  cloud_out.width = cloud2_.width;
  cloud_out.is_dense = cloud2_.is_dense;

  cloud_out.points.resize (cloud_out.height * cloud_out.width);
  
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  int depth_idx = 0;
  float constant = 0.001 / rgb_focal_length_VGA_;

  unsigned depth_step = depth_msg->width / cloud_out.width;
  unsigned depth_skip = (depth_msg->height / cloud_out.height - 1) * depth_msg->width;

  int centerX = cloud2_.width >> 1;
  int centerY = cloud2_.height >> 1;
  constant *= depth_step;

  unsigned char* rgb_buffer = (unsigned char*)&rgb_image_.data[0];
  RGBValue color;
  color.Alpha = 0;

  int color_idx = 0;
  unsigned color_step = 3 * rgb_image_.width / cloud_out.width;
  unsigned color_skip = 3 * (rgb_image_.height / cloud_out.height - 1) * rgb_image_.width;

  /// @todo Right now we always copy the raw depth data into a sensor_msgs/Image. In principle this
  /// isn't completely avoidable, as the synchronizer may wait for the next depth image before choosing
  /// the old data, which is overwritten by that point. In practice though we should optimize to avoid
  /// this almost always.
  const uint16_t* depth_md = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);

  for (int v = 0; v < (int)cloud_out.height; ++v, depth_idx += depth_skip, color_idx += color_skip)
  {
    for (int u = 0; u < (int)cloud_out.width; ++u, depth_idx += depth_step, color_idx += color_step)
    {
      pcl::PointXYZRGB& pt = cloud_out (u, v);
      
      // Check for invalid measurements
      if (depth_md[depth_idx] == 0 ||
          depth_md[depth_idx] == no_sample_value_ ||
          depth_md[depth_idx] == shadow_value_)
      {
        // not valid
        pt.x = pt.y = pt.z = bad_point;
      }
      else
      {
        // Fill in XYZ
        pt.x = (u - centerX) * depth_md[depth_idx] * constant;
        pt.y = (v - centerY) * depth_md[depth_idx] * constant;
        pt.z = depth_md[depth_idx] * 0.001;
      }

      // Fill in color (whether valid or invalid)
      color.Red   = rgb_buffer[color_idx];
      color.Green = rgb_buffer[color_idx + 1];
      color.Blue  = rgb_buffer[color_idx + 2];
      pt.rgb = color.float_value;
    }
  }

  cloud_out.header.stamp = max (depth_msg->header.stamp, rgb_msg->header.stamp);
  cloud_out.header.frame_id = rgb_msg->header.frame_id;
  cloud_out.header.seq = max (depth_msg->header.seq, rgb_msg->header.seq);

  pub_depth_points2_.publish (cloud_out.makeShared ());
}

void OpenNIDriver::configCb (Config &config, uint32_t level)
{
  // highres images for the primesense device are not supported yet => fall back to VGA
  if (config.device_type == OpenNI_Primesense && config.image_resolution == OpenNI_SXGA_15Hz)
  {
    config.image_resolution = OpenNI_VGA_30Hz;
  }

  // if our pointcloud is XYZRGB then the image resolution must be at least the same as the point cloud resolution
  if (config.point_cloud_resolution > config.image_resolution && config.point_cloud_type)
  {
    config.image_resolution = config.point_cloud_resolution;
  }

  config_ = config;
  updateDeviceSettings();
}

bool OpenNIDriver::updateDeviceSettings()
{
  unsigned image_width, image_height;
  unsigned stream_width, stream_height, stream_fps;

  stream_width   = XN_VGA_X_RES;
  stream_height  = XN_VGA_Y_RES;
  stream_fps     = 30;

  switch( config_.image_resolution )
  {
    case OpenNI_QQVGA_30Hz:
            image_width    = 160;
            image_height   = 120;
      break;

    case OpenNI_QVGA_30Hz:
            image_width    = 320;
            image_height   = 240;
      break;

    case OpenNI_VGA_30Hz:
            image_width    = XN_VGA_X_RES;
            image_height   = XN_VGA_Y_RES;
      break;

    case OpenNI_SXGA_15Hz:
            stream_width   = 1280;
            stream_height  = 1024;
            stream_fps     = 15;
            image_width    = 1280;
            image_height   = 1024;
      break;

    default:ROS_WARN("Unknwon image resolution - falling back to VGA@30Hz");
            image_width    = XN_VGA_X_RES;
            image_height   = XN_VGA_Y_RES;
      break;
  }

  switch( config_.point_cloud_resolution )
  {
    case OpenNI_QQVGA_30Hz:
            cloud2_.height = 120;
            cloud2_.width  = 160;
            break;

    case OpenNI_QVGA_30Hz:
            cloud2_.height = 240;
            cloud2_.width  = 320;
            break;

    case OpenNI_VGA_30Hz:
            cloud2_.height = XN_VGA_Y_RES;
            cloud2_.width  = XN_VGA_X_RES;
            break;
            
    default: ROS_WARN("Unknwon point cloud size - falling back to VGA");
            cloud2_.height = XN_VGA_Y_RES;
            cloud2_.width  = XN_VGA_X_RES;
            break;
  }

  XnStatus status;
  if (config_.point_cloud_type == OpenNI_XYZRGB)
  {
    cloud2_.fields.resize( 4 );
    cloud2_.fields[0].name = "x";
    cloud2_.fields[1].name = "y";
    cloud2_.fields[2].name = "z";
    cloud2_.fields[3].name = "rgb";
  }
  else
  {
    cloud2_.fields.resize( 3 );
    cloud2_.fields[0].name = "x";
    cloud2_.fields[1].name = "y";
    cloud2_.fields[2].name = "z";
  }

  // Set all the fields types accordingly
  int offset = 0;
  for (size_t s = 0; s < cloud2_.fields.size (); ++s, offset += sizeof(float))
  {
    cloud2_.fields[s].offset   = offset;
    cloud2_.fields[s].count    = 1;
    cloud2_.fields[s].datatype = sensor_msgs::PointField::FLOAT32;
  }

  cloud2_.point_step = offset;
  cloud2_.row_step   = cloud2_.point_step * cloud2_.width; /// @todo *offset?
  cloud2_.data.resize (cloud2_.row_step   * cloud2_.height);
  cloud2_.is_dense = false;

  /// @todo Inline function to turn resolution enum into height/width
  // Assemble the depth image data
  switch( config_.disparity_resolution )
  {
    case OpenNI_QQVGA_30Hz:
        disp_image_.image.height = 120;
        disp_image_.image.width = 160;
      break;

    case OpenNI_QVGA_30Hz:
        disp_image_.image.height = 240;
        disp_image_.image.width = 320;
      break;

    case OpenNI_VGA_30Hz:
        disp_image_.image.height = 480;
        disp_image_.image.width = 640;
      break;

    default: ROS_WARN("Unknwon disparity resolution - falling back to VGA");
        disp_image_.image.height = XN_VGA_Y_RES;
        disp_image_.image.width = XN_VGA_X_RES;
      break;
  }
  
  disp_image_.image.step =  disp_image_.image.width * sizeof (float);
  disp_image_.image.data.resize (disp_image_.image.step * disp_image_.image.height);

  // Assemble the color image data
  rgb_image_.height = image_height;
  rgb_image_.width = image_width;
  rgb_info_.header.frame_id = rgb_image_.header.frame_id; 
  rgb_image_.encoding = sensor_msgs::image_encodings::RGB8;
  rgb_image_.data.resize (image_width * image_height * 3);
  rgb_image_.step = image_width * 3;


  // Assemble the gray image data
  gray_image_.height = image_height;
  gray_image_.width = image_width;
  gray_image_.encoding = sensor_msgs::image_encodings::MONO8;
  gray_image_.data.resize (image_width * image_height);
  gray_image_.step = image_width;

  XnMapOutputMode mode;
  XnStatus rc_;
  if (!depth_generator_.IsValid())
  {
    rc_ = depth_generator_.Create (context_);

    if (rc_ != XN_STATUS_OK)
    {
      ROS_ERROR ("[OpenNIDriver] Failed to create DepthGenerator: %s\n Please see http://www.ros.org/wiki/ni/Troubleshooting for more info.", xnGetStatusString (rc_));
      return (false);
    }

    // Set the correct mode on the depth/image generator
    mode.nXRes = depth_stream_width;
    mode.nYRes = depth_stream_height;
    mode.nFPS  = depth_stream_fps;
    if (depth_generator_.SetMapOutputMode (mode) != XN_STATUS_OK)
    {
      ROS_ERROR("[OpenNIDriver] Failed to set depth output mode");
      return (false);
    }

    XnDouble pixel_size;
    // Read parameters from the camera
    if (depth_generator_.GetRealProperty ("ZPPS", pixel_size) != XN_STATUS_OK)
      ROS_ERROR ("[OpenNIDriver] Could not read pixel size!");

    // pixel size @ VGA = pixel size @ SXGA x 2
    pixel_size *= 2.0;

    // focal length of IR camera in pixels for VGA resolution
    XnUInt64 depth_focal_length_VGA;
    if (depth_generator_.GetIntProperty ("ZPD", depth_focal_length_VGA) != XN_STATUS_OK)
      ROS_ERROR ("[OpenNIDriver] Could not read virtual plane distance!");

    if (depth_generator_.GetRealProperty ("LDDIS", baseline_) != XN_STATUS_OK)
      ROS_ERROR ("[OpenNIDriver] Could not read base line!");

    // baseline from cm -> meters
    baseline_ *= 0.01;

    //focal length from mm -> pixels (valid for 640x480)
    depth_focal_length_VGA_ = (double)depth_focal_length_VGA/pixel_size;

    if (depth_generator_.GetIntProperty ("ShadowValue", shadow_value_) != XN_STATUS_OK)
      ROS_WARN ("[OpenNIDriver] Could not read shadow value!");

    if (depth_generator_.GetIntProperty ("NoSampleValue", no_sample_value_) != XN_STATUS_OK)
      ROS_WARN ("[OpenNIDriver] Could not read no sample value!");
  }

  // No distortion (yet!)
#if ROS_VERSION_MINIMUM(1, 3, 0)
  depth_info_.D = std::vector<double>( 5, 0.0 );
  depth_info_.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
#else
  depth_info_.D.assign( 0.0 );
#endif
  depth_info_.K.assign( 0.0 );
  depth_info_.R.assign( 0.0 );
  depth_info_.P.assign( 0.0 );

  depth_info_.K[0] = depth_info_.K[4] = depth_focal_length_VGA_ * (double)disp_image_.image.width / (double)depth_stream_width;
  depth_info_.K[2] = disp_image_.image.width >> 1;
  depth_info_.K[5] = disp_image_.image.height >> 1;
  depth_info_.K[8] = 1.0;

  // no rotation: identity
  depth_info_.R[0] = depth_info_.R[4] = depth_info_.R[8] = 1.0;

  // no rotation, no translation => P=K(I|0)=(K|0)
  depth_info_.P[0]    = depth_info_.P[5] = depth_info_.K[0];
  depth_info_.P[2]    = depth_info_.K[2];
  depth_info_.P[6]    = depth_info_.K[5];
  depth_info_.P[10]   = 1.0;
  depth_info_.width   = disp_image_.image.width;
  depth_info_.height  = disp_image_.image.height;
  depth_info_.header.frame_id = IR_frame_id_;

  // got baseline update disparity image
  disp_image_.image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_image_.T = baseline_;
  
  /// @todo Compute these values from DepthGenerator::GetDeviceMaxDepth() and the like
  disp_image_.min_disparity = 0.0;
  disp_image_.max_disparity = disp_image_.T * disp_image_.f / 0.3;
  disp_image_.delta_d = 0.125;

  // No distortion (yet!)
#if ROS_VERSION_MINIMUM(1, 3, 0)
  rgb_info_.D = std::vector<double>( 5, 0.0 );
  rgb_info_.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
#else
  rgb_info_.D.assign( 0.0 );
#endif
  rgb_info_.K.assign( 0.0 );
  rgb_info_.R.assign( 0.0 );
  rgb_info_.P.assign( 0.0 );

  rgb_info_.K[0] = rgb_info_.K[4] = rgb_focal_length_VGA_ * (double)image_width / 640.0;
  //set the camera center so that it is consistent with the depth_image center
  rgb_info_.K[2] = (depth_info_.width * (double)image_width / 640.0) / 2.0; 
  rgb_info_.K[5] = (depth_info_.height * (double)image_width / 640.0) / 2.0;
  rgb_info_.K[8] = 1.0;

  // no rotation: identity
  rgb_info_.R[0] = rgb_info_.R[4] = rgb_info_.R[8] = 1.0;

  // no rotation, no translation => P=K(I|0)=(K|0)
  rgb_info_.P[0]    = rgb_info_.P[5] = rgb_info_.K[0];
  rgb_info_.P[2]    = rgb_info_.K[2];
  rgb_info_.P[6]    = rgb_info_.K[5];
  rgb_info_.P[10]   = 1.0;
  rgb_info_.width   = image_width;
  rgb_info_.height  = image_height;

  depth_generator_.GetAlternativeViewPointCap ().ResetViewPoint ();

  if (!image_generator_.IsValid())
  {
    rc_ = image_generator_.Create (context_);

    if (rc_ != XN_STATUS_OK)
    {
      ROS_ERROR ("[OpenNIDriver] Failed to create ImageGenerator: %s", xnGetStatusString (rc_));
      return (false);
    }
  }

  mode.nXRes = stream_width;
  mode.nYRes = stream_height;
  mode.nFPS  = stream_fps;
  if (image_generator_.SetMapOutputMode (mode) != XN_STATUS_OK)
  {
    ROS_ERROR("[OpenNIDriver] Failed to set image output mode");
    return (false);
  }

  if( config_.device_type == OpenNI_Primesense )
  {
    // 5 => YUV422
    if (image_generator_.SetIntProperty ("InputFormat", 5) != XN_STATUS_OK)
      ROS_ERROR ("[OpenNIDriver] Error setting the image input format to Uncompressed 8-bit BAYER!");

    // RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS
    if (depth_generator_.SetIntProperty ("RegistrationType", 1) != XN_STATUS_OK)
      ROS_WARN ("[OpenNIDriver] Error enabling registration!");

    if (image_generator_.SetPixelFormat(XN_PIXEL_FORMAT_YUV422 )!= XN_STATUS_OK)
    {
      ROS_ERROR("[OpenNIDriver] Failed to set image pixel format to YUV422");
      return (false);
    }
  }
  else if( config_.device_type == OpenNI_Kinect )
  {
    // InputFormat should be 6 = uncompressed Bayer for Kinect
    if (image_generator_.SetIntProperty ("InputFormat", 6) != XN_STATUS_OK)
      ROS_ERROR ("[OpenNIDriver] Error setting the image input format to Uncompressed 8-bit BAYER!");

    // RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS
    if (depth_generator_.SetIntProperty ("RegistrationType", 2) != XN_STATUS_OK)
      ROS_WARN ("[OpenNIDriver] Error enabling registration!");

    // Grayscale: bypass debayering -> gives us bayer pattern!
    if (image_generator_.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT )!= XN_STATUS_OK)
    {
      ROS_ERROR("[OpenNIDriver] Failed to set image pixel format to 8bit-grayscale");
      return (false);
    }
  }
  else
  {
    ROS_ERROR("[OpenNIDriver] unknown device type");
    return (false);
  }

  if (config_.point_cloud_type == OpenNI_XYZ_unregistered) // not registered pc
  {
    status = depth_generator_.GetAlternativeViewPointCap().ResetViewPoint();
    if (status != XN_STATUS_OK)
    {
      ROS_ERROR ("[OpenNIDriver::spin] Error in switching off registering on depth stream: %s", xnGetStatusString (status));
      return (false);
    }

    // unregistered -> all depth information are in the IR camera coordinate system
    disp_image_.header.frame_id = IR_frame_id_;
    disp_image_.f = depth_focal_length_VGA_;
  }
  else
  {
    status = depth_generator_.GetAlternativeViewPointCap().SetViewPoint( image_generator_ );
    if (status != XN_STATUS_OK)
    {
      ROS_ERROR ("[OpenNIDriver::spin] Error in switching on depth stream registration: %s", xnGetStatusString (status));
      return (false);
    }

    // registered => all depth information are in the RGB camera coordinate system, as well as the camera parameters from RGB are valid for the depth image
    depth_info_ = rgb_info_;
    disp_image_.header.frame_id = rgb_frame_id_;
    disp_image_.f = rgb_focal_length_VGA_;
  }

  return true;
}

void OpenNIDriver::bayer2RGB ( const xn::ImageMetaData& bayer, sensor_msgs::Image& image, int method ) const
{
  if (bayer.XRes() == image.width && bayer.YRes() == image.height)
  {
    register const XnUInt8 *bayer_pixel = bayer.Data();
    register unsigned yIdx, xIdx;

    int line_step = image.width;
    int line_step2 = image.width << 1;

    int rgb_line_step  = line_step * 3;             // previous color line
    register unsigned char *rgb_pixel = (unsigned char *)&image.data[0];

    if (method == OpenNI_Bilinear)
    {
      // first two pixel values for first two lines
      // Bayer         0 1 2
      //         0     G r g
      // line_step     b g b
      // line_step2    g r g

      rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
      rgb_pixel[1] = bayer_pixel[0];    // green pixel
      rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

      // Bayer         0 1 2
      //         0     g R g
      // line_step     b g b
      // line_step2    g r g
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
      rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

      // BGBG line
      // Bayer         0 1 2
      //         0     g r g
      // line_step     B g b
      // line_step2    g r g
      rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // pixel (1, 1)  0 1 2
      //         0     g r g
      // line_step     b G b
      // line_step2    g r g
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

      rgb_pixel += 6;
      bayer_pixel += 2;
      // rest of the first two lines
      for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer        -1 0 1 2
        //           0   r G r g
        //   line_step   g b g b
        // line_step2    r g r g
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = bayer_pixel[line_step + 1];

        // Bayer        -1 0 1 2
        //          0    r g R g
        //  line_step    g b g b
        // line_step2    r g r g
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

        // BGBG line
        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g B g b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g b G b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer        -1 0 1
      //           0   r G r
      //   line_step   g b g
      // line_step2    r g r
      rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
      rgb_pixel[1] = bayer_pixel[0];
      rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

      // Bayer        -1 0 1
      //          0    r g R
      //  line_step    g b g
      // line_step2    r g r
      rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
      //rgb_pixel[5] = bayer_pixel[line_step];

      // BGBG line
      // Bayer        -1 0 1
      //          0    r g r
      //  line_step    g B g
      // line_step2    r g r
      rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
      rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer         -1 0 1
      //         0      r g r
      // line_step      g b G
      // line_step2     r g r
      rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

      bayer_pixel += line_step + 2;
      rgb_pixel += rgb_line_step + 6;

      // main processing
      for (yIdx = 2; yIdx < image.height-2; yIdx += 2)
      {
        // first two pixel values
        // Bayer         0 1 2
        //        -1     b g b
        //         0     G r g
        // line_step     b g b
        // line_step2    g r g

        rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
        rgb_pixel[1] = bayer_pixel[0];    // green pixel
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] ); // blue;

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g R g
        // line_step     b g b
        // line_step2    g r g
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

        // BGBG line
        // Bayer         0 1 2
        //         0     g r g
        // line_step     B g b
        // line_step2    g r g
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // pixel (1, 1)  0 1 2
        //         0     g r g
        // line_step     b G b
        // line_step2    g r g
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

        rgb_pixel += 6;
        bayer_pixel += 2;
        // continue with rest of the line
        for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer        -1 0 1 2
          //          -1   g b g b
          //           0   r G r g
          //   line_step   g b g b
          // line_step2    r g r g
          rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
          rgb_pixel[1] = bayer_pixel[0];
          rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

          // Bayer        -1 0 1 2
          //          -1   g b g b
          //          0    r g R g
          //  line_step    g b g b
          // line_step2    r g r g
          rgb_pixel[3] = bayer_pixel[1];
          rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
          rgb_pixel[5] = AVG4( bayer_pixel[-line_step], bayer_pixel[2-line_step], bayer_pixel[line_step], bayer_pixel[line_step+2]);

          // BGBG line
          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g B g b
          // line_step2     r g r g
          rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1], bayer_pixel[line_step2+1], bayer_pixel[-1], bayer_pixel[line_step2-1] );
          rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0], bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
          rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g b G b
          // line_step2     r g r g
          rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
          rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
          rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
        }

        // last two pixels of the line
        // last two pixel values for first two lines
        // GRGR line
        // Bayer        -1 0 1
        //           0   r G r
        //   line_step   g b g
        // line_step2    r g r
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

        // Bayer        -1 0 1
        //          0    r g R
        //  line_step    g b g
        // line_step2    r g r
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
        //rgb_pixel[5] = bayer_pixel[line_step];

        // BGBG line
        // Bayer        -1 0 1
        //          0    r g r
        //  line_step    g B g
        // line_step2    r g r
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1
        //         0      r g r
        // line_step      g b G
        // line_step2     r g r
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

        bayer_pixel += line_step + 2;
        rgb_pixel += rgb_line_step + 6;
      }

      //last two lines
      // Bayer         0 1 2
      //        -1     b g b
      //         0     G r g
      // line_step     b g b

      rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
      rgb_pixel[1] = bayer_pixel[0];    // green pixel
      rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g R g
      // line_step     b g b
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
      rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

      // BGBG line
      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     B g b
      //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
      rgb_pixel[rgb_line_step + 1] = AVG( bayer_pixel[0] , bayer_pixel[line_step+1] );
      rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     b G b
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

      rgb_pixel += 6;
      bayer_pixel += 2;
      // rest of the last two lines
      for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r G r g
        // line_step    g b g b
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g R g
        // line_step    g b g b
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[-line_step+2] );

        // BGBG line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g B g b
        rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[-1], bayer_pixel[1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];


        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g b G b
        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r G r
      // line_step    g b g
      rgb_pixel[rgb_line_step    ] = rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
      rgb_pixel[1] = bayer_pixel[0];
      rgb_pixel[5] = rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g R
      // line_step    g b g
      rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[line_step+1], bayer_pixel[-line_step+1] );
      //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

      // BGBG line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g B g
      //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
      rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
      rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g b G
      //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
    }
    else if (method == OpenNI_EdgeAware)
    {
      int dh, dv;

      // first two pixel values for first two lines
      // Bayer         0 1 2
      //         0     G r g
      // line_step     b g b
      // line_step2    g r g

      rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
      rgb_pixel[1] = bayer_pixel[0];    // green pixel
      rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

      // Bayer         0 1 2
      //         0     g R g
      // line_step     b g b
      // line_step2    g r g
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
      rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

      // BGBG line
      // Bayer         0 1 2
      //         0     g r g
      // line_step     B g b
      // line_step2    g r g
      rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // pixel (1, 1)  0 1 2
      //         0     g r g
      // line_step     b G b
      // line_step2    g r g
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

      rgb_pixel += 6;
      bayer_pixel += 2;
      // rest of the first two lines
      for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer        -1 0 1 2
        //           0   r G r g
        //   line_step   g b g b
        // line_step2    r g r g
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = bayer_pixel[line_step + 1];

        // Bayer        -1 0 1 2
        //          0    r g R g
        //  line_step    g b g b
        // line_step2    r g r g
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

        // BGBG line
        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g B g b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g b G b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer        -1 0 1
      //           0   r G r
      //   line_step   g b g
      // line_step2    r g r
      rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
      rgb_pixel[1] = bayer_pixel[0];
      rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

      // Bayer        -1 0 1
      //          0    r g R
      //  line_step    g b g
      // line_step2    r g r
      rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
      //rgb_pixel[5] = bayer_pixel[line_step];

      // BGBG line
      // Bayer        -1 0 1
      //          0    r g r
      //  line_step    g B g
      // line_step2    r g r
      rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
      rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer         -1 0 1
      //         0      r g r
      // line_step      g b G
      // line_step2     r g r
      rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

      bayer_pixel += line_step + 2;
      rgb_pixel += rgb_line_step + 6;
      // main processing
      for (yIdx = 2; yIdx < image.height-2; yIdx += 2)
      {
        // first two pixel values
        // Bayer         0 1 2
        //        -1     b g b
        //         0     G r g
        // line_step     b g b
        // line_step2    g r g

        rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
        rgb_pixel[1] = bayer_pixel[0];    // green pixel
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] ); // blue;

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g R g
        // line_step     b g b
        // line_step2    g r g
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

        // BGBG line
        // Bayer         0 1 2
        //         0     g r g
        // line_step     B g b
        // line_step2    g r g
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // pixel (1, 1)  0 1 2
        //         0     g r g
        // line_step     b G b
        // line_step2    g r g
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

        rgb_pixel += 6;
        bayer_pixel += 2;
        // continue with rest of the line
        for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer        -1 0 1 2
          //          -1   g b g b
          //           0   r G r g
          //   line_step   g b g b
          // line_step2    r g r g
          rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
          rgb_pixel[1] = bayer_pixel[0];
          rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

          // Bayer        -1 0 1 2
          //          -1   g b g b
          //          0    r g R g
          //  line_step    g b g b
          // line_step2    r g r g

          dh = abs( bayer_pixel[0] - bayer_pixel[2] );
          dv = abs( bayer_pixel[-line_step+1] - bayer_pixel[line_step+1] );

          if( dh > dv )
            rgb_pixel[4] = AVG( bayer_pixel[-line_step+1], bayer_pixel[line_step+1] );
          else if( dv > dh )
            rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[2] );
          else
            rgb_pixel[4] = AVG4( bayer_pixel[-line_step+1], bayer_pixel[line_step+1], bayer_pixel[0], bayer_pixel[2] );

          rgb_pixel[3] = bayer_pixel[1];
          rgb_pixel[5] = AVG4( bayer_pixel[-line_step], bayer_pixel[2-line_step], bayer_pixel[line_step], bayer_pixel[line_step+2]);

          // BGBG line
          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g B g b
          // line_step2     r g r g
          rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1], bayer_pixel[line_step2+1], bayer_pixel[-1], bayer_pixel[line_step2-1] );
          rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

          dv = abs( bayer_pixel[0] - bayer_pixel[line_step2] );
          dh = abs( bayer_pixel[line_step-1] - bayer_pixel[line_step+1] );

          if( dv > dh )
            rgb_pixel[rgb_line_step + 1] = AVG( bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
          else if( dh > dv )
            rgb_pixel[rgb_line_step + 1] = AVG( bayer_pixel[0], bayer_pixel[line_step2] );
          else
            rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0], bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );

          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g b G b
          // line_step2     r g r g
          rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
          rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
          rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
        }

        // last two pixels of the line
        // last two pixel values for first two lines
        // GRGR line
        // Bayer        -1 0 1
        //           0   r G r
        //   line_step   g b g
        // line_step2    r g r
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

        // Bayer        -1 0 1
        //          0    r g R
        //  line_step    g b g
        // line_step2    r g r
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
        //rgb_pixel[5] = bayer_pixel[line_step];

        // BGBG line
        // Bayer        -1 0 1
        //          0    r g r
        //  line_step    g B g
        // line_step2    r g r
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1
        //         0      r g r
        // line_step      g b G
        // line_step2     r g r
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

        bayer_pixel += line_step + 2;
        rgb_pixel += rgb_line_step + 6;
      }

      //last two lines
      // Bayer         0 1 2
      //        -1     b g b
      //         0     G r g
      // line_step     b g b

      rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
      rgb_pixel[1] = bayer_pixel[0];    // green pixel
      rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g R g
      // line_step     b g b
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
      rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

      // BGBG line
      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     B g b
      //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
      rgb_pixel[rgb_line_step + 1] = AVG( bayer_pixel[0] , bayer_pixel[line_step+1] );
      rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     b G b
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

      rgb_pixel += 6;
      bayer_pixel += 2;
      // rest of the last two lines
      for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r G r g
        // line_step    g b g b
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g R g
        // line_step    g b g b
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[-line_step+2] );

        // BGBG line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g B g b
        rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[-1], bayer_pixel[1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];


        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g b G b
        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r G r
      // line_step    g b g
      rgb_pixel[rgb_line_step    ] = rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
      rgb_pixel[1] = bayer_pixel[0];
      rgb_pixel[5] = rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g R
      // line_step    g b g
      rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[line_step+1], bayer_pixel[-line_step+1] );
      //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

      // BGBG line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g B g
      //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
      rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
      rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g b G
      //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
    }
    else if( method == OpenNI_EdgeAwareWeighted )
    {
      int dh, dv;

      // first two pixel values for first two lines
      // Bayer         0 1 2
      //         0     G r g
      // line_step     b g b
      // line_step2    g r g

      rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
      rgb_pixel[1] = bayer_pixel[0];    // green pixel
      rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

      // Bayer         0 1 2
      //         0     g R g
      // line_step     b g b
      // line_step2    g r g
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
      rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

      // BGBG line
      // Bayer         0 1 2
      //         0     g r g
      // line_step     B g b
      // line_step2    g r g
      rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // pixel (1, 1)  0 1 2
      //         0     g r g
      // line_step     b G b
      // line_step2    g r g
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

      rgb_pixel += 6;
      bayer_pixel += 2;
      // rest of the first two lines
      for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer        -1 0 1 2
        //           0   r G r g
        //   line_step   g b g b
        // line_step2    r g r g
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = bayer_pixel[line_step + 1];

        // Bayer        -1 0 1 2
        //          0    r g R g
        //  line_step    g b g b
        // line_step2    r g r g
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

        // BGBG line
        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g B g b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g b G b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer        -1 0 1
      //           0   r G r
      //   line_step   g b g
      // line_step2    r g r
      rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
      rgb_pixel[1] = bayer_pixel[0];
      rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

      // Bayer        -1 0 1
      //          0    r g R
      //  line_step    g b g
      // line_step2    r g r
      rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
      //rgb_pixel[5] = bayer_pixel[line_step];

      // BGBG line
      // Bayer        -1 0 1
      //          0    r g r
      //  line_step    g B g
      // line_step2    r g r
      rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
      rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer         -1 0 1
      //         0      r g r
      // line_step      g b G
      // line_step2     r g r
      rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

      bayer_pixel += line_step + 2;
      rgb_pixel += rgb_line_step + 6;
      // main processing
      for (yIdx = 2; yIdx < image.height-2; yIdx += 2)
      {
        // first two pixel values
        // Bayer         0 1 2
        //        -1     b g b
        //         0     G r g
        // line_step     b g b
        // line_step2    g r g

        rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
        rgb_pixel[1] = bayer_pixel[0];    // green pixel
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] ); // blue;

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g R g
        // line_step     b g b
        // line_step2    g r g
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

        // BGBG line
        // Bayer         0 1 2
        //         0     g r g
        // line_step     B g b
        // line_step2    g r g
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // pixel (1, 1)  0 1 2
        //         0     g r g
        // line_step     b G b
        // line_step2    g r g
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

        rgb_pixel += 6;
        bayer_pixel += 2;
        // continue with rest of the line
        for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer        -1 0 1 2
          //          -1   g b g b
          //           0   r G r g
          //   line_step   g b g b
          // line_step2    r g r g
          rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
          rgb_pixel[1] = bayer_pixel[0];
          rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

          // Bayer        -1 0 1 2
          //          -1   g b g b
          //          0    r g R g
          //  line_step    g b g b
          // line_step2    r g r g

          dh = abs( bayer_pixel[0] - bayer_pixel[2] );
          dv = abs( bayer_pixel[-line_step+1] - bayer_pixel[line_step+1] );

          if( dv == 0 && dh == 0 )
            rgb_pixel[4] = AVG4( bayer_pixel[1-line_step], bayer_pixel[1+line_step], bayer_pixel[0], bayer_pixel[2] );
          else
            rgb_pixel[4] = WAVG4( bayer_pixel[1-line_step], bayer_pixel[1+line_step], bayer_pixel[0], bayer_pixel[2], dh, dv );
          rgb_pixel[3] = bayer_pixel[1];
          rgb_pixel[5] = AVG4( bayer_pixel[-line_step], bayer_pixel[2-line_step], bayer_pixel[line_step], bayer_pixel[line_step+2]);

          // BGBG line
          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g B g b
          // line_step2     r g r g
          rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1], bayer_pixel[line_step2+1], bayer_pixel[-1], bayer_pixel[line_step2-1] );
          rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

          dv = abs( bayer_pixel[0] - bayer_pixel[line_step2] );
          dh = abs( bayer_pixel[line_step-1] - bayer_pixel[line_step+1] );

          if( dv == 0 && dh == 0 )
            rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0], bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
          else
            rgb_pixel[rgb_line_step + 1] = WAVG4( bayer_pixel[0], bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1], dh, dv );

          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g b G b
          // line_step2     r g r g
          rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
          rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
          rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
        }

        // last two pixels of the line
        // last two pixel values for first two lines
        // GRGR line
        // Bayer        -1 0 1
        //           0   r G r
        //   line_step   g b g
        // line_step2    r g r
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

        // Bayer        -1 0 1
        //          0    r g R
        //  line_step    g b g
        // line_step2    r g r
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
        //rgb_pixel[5] = bayer_pixel[line_step];

        // BGBG line
        // Bayer        -1 0 1
        //          0    r g r
        //  line_step    g B g
        // line_step2    r g r
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1
        //         0      r g r
        // line_step      g b G
        // line_step2     r g r
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

        bayer_pixel += line_step + 2;
        rgb_pixel += rgb_line_step + 6;
      }

      //last two lines
      // Bayer         0 1 2
      //        -1     b g b
      //         0     G r g
      // line_step     b g b

      rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
      rgb_pixel[1] = bayer_pixel[0];    // green pixel
      rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g R g
      // line_step     b g b
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
      rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

      // BGBG line
      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     B g b
      //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
      rgb_pixel[rgb_line_step + 1] = AVG( bayer_pixel[0] , bayer_pixel[line_step+1] );
      rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     b G b
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

      rgb_pixel += 6;
      bayer_pixel += 2;
      // rest of the last two lines
      for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r G r g
        // line_step    g b g b
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g R g
        // line_step    g b g b
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[-line_step+2] );

        // BGBG line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g B g b
        rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[-1], bayer_pixel[1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];


        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g b G b
        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r G r
      // line_step    g b g
      rgb_pixel[rgb_line_step    ] = rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
      rgb_pixel[1] = bayer_pixel[0];
      rgb_pixel[5] = rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g R
      // line_step    g b g
      rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[line_step+1], bayer_pixel[-line_step+1] );
      //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

      // BGBG line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g B g
      //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
      rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
      rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g b G
      //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
    }
    else
    {
      ROS_WARN("[OpenNIDriver::Debayering] Unknown Debayering method %d", method );
    }
  }
  else
  {
    // get each or each 2nd pixel group to find rgb values!
    register unsigned bayerXStep = bayer.XRes() / image.width;
    register unsigned bayerYSkip = (bayer.YRes() / image.height - 1) * bayer.XRes();

    // Downsampling and debayering at once
    register const XnUInt8* bayer_buffer = bayer.Data();
    register unsigned char* rgb_buffer = (unsigned char*)&image.data[0];

    for( register unsigned yIdx = 0; yIdx < image.height; ++yIdx, bayer_buffer += bayerYSkip ) // skip a line
    {
      for( register unsigned xIdx = 0; xIdx < image.width; ++xIdx, rgb_buffer += 3, bayer_buffer += bayerXStep )
      {
        rgb_buffer[ 2 ] = bayer_buffer[ bayer.XRes() ];
        rgb_buffer[ 1 ] = AVG( bayer_buffer[0], bayer_buffer[ bayer.XRes() + 1] );
        rgb_buffer[ 0 ] = bayer_buffer[ 1 ];
      }
    }
  }
}

void OpenNIDriver::bayer2Gray ( const xn::ImageMetaData& bayer, sensor_msgs::Image& image, int method ) const
{
  if (bayer.XRes() == image.width && bayer.YRes() == image.height)
  {
    unsigned char* gray_pixel = (unsigned char *)&image.data[0];
    const XnUInt8 *bayer_pixel = bayer.Data();
    int line_skip = image.width;

    if (method == OpenNI_Bilinear)
    {
      // first line GRGRGR
      for (register unsigned xIdx = 0; xIdx < image.width-2; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
      {
        gray_pixel[0] = bayer_pixel[0];            // green pixel
        gray_pixel[1] = AVG3(bayer_pixel[0], bayer_pixel[2], bayer_pixel[1+line_skip]); // interpolated green pixel
      }
      gray_pixel[0] = bayer_pixel[0];
      gray_pixel[1] = AVG( bayer_pixel[0], bayer_pixel[1+line_skip]);
      gray_pixel += 2;
      bayer_pixel += 2;
      
      for (register unsigned yIdx = 1; yIdx < image.height-1; yIdx += 2)
      {
        // blue line
        gray_pixel[0] = AVG3(bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[1]);
        gray_pixel[1] = bayer_pixel[1];
        gray_pixel += 2;
        bayer_pixel += 2;
        for (register unsigned xIdx = 2; xIdx < image.width; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
        {
          gray_pixel[0] = AVG4(bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[-1], bayer_pixel[1]);
          gray_pixel[1] = bayer_pixel[1];
        }

        // red line
        for (register unsigned xIdx = 0; xIdx < image.width-2; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
        {
          gray_pixel[0] = bayer_pixel[0];            // green pixel
          gray_pixel[1] = AVG4(bayer_pixel[0], bayer_pixel[2], bayer_pixel[-line_skip+1], bayer_pixel[line_skip+1]); // interpolated green pixel
        }
        gray_pixel[0] = bayer_pixel[0];
        gray_pixel[1] = AVG3(bayer_pixel[-line_skip+1], bayer_pixel[line_skip+1],bayer_pixel[-1]);
        gray_pixel += 2;
        bayer_pixel += 2;
      }
      
      // last line BGBGBG
      gray_pixel[0] = AVG(bayer_pixel[1], bayer_pixel[-line_skip]);
      gray_pixel[1] = bayer_pixel[1];
      gray_pixel += 2;
      bayer_pixel += 2;
      for (register unsigned xIdx = 2; xIdx < image.width; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
      {
        gray_pixel[0] = AVG3( bayer_pixel[-1], bayer_pixel[1], bayer_pixel[-line_skip] );
        gray_pixel[1] = bayer_pixel[1];
      }
    }
    else if (method == OpenNI_EdgeAware)
    {
      int dv, dh;
      // first line GRGRGR
      for (register unsigned xIdx = 0; xIdx < image.width-2; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
      {
        gray_pixel[0] = bayer_pixel[0];            // green pixel
        gray_pixel[1] = AVG3(bayer_pixel[0], bayer_pixel[2], bayer_pixel[1+line_skip]); // interpolated green pixel
      }
      gray_pixel[0] = bayer_pixel[0];
      gray_pixel[1] = AVG( bayer_pixel[0], bayer_pixel[1+line_skip]);
      gray_pixel += 2;
      bayer_pixel += 2;

      for (register unsigned yIdx = 1; yIdx < image.height-1; yIdx += 2)
      {
        // blue line
        gray_pixel[0] = AVG3(bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[1]);
        gray_pixel[1] = bayer_pixel[1];
        gray_pixel += 2;
        bayer_pixel += 2;
        for (register unsigned xIdx = 2; xIdx < image.width; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
        {
          dv = abs( bayer_pixel[-line_skip] - bayer_pixel[line_skip] );
          dh = abs( bayer_pixel[-1] - bayer_pixel[1] );
          if (dh > dv)
            gray_pixel[0] = AVG(bayer_pixel[-line_skip], bayer_pixel[line_skip]);
          else if (dv > dh)
            gray_pixel[0] = AVG(bayer_pixel[-1], bayer_pixel[1]);
          else
            gray_pixel[0] = AVG4(bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[-1], bayer_pixel[1]);

          gray_pixel[1] = bayer_pixel[1];
        }

        // red line
        for (register unsigned xIdx = 0; xIdx < image.width-2; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
        {
          gray_pixel[0] = bayer_pixel[0];

          dv = abs( bayer_pixel[1-line_skip] - bayer_pixel[1+line_skip] );
          dh = abs( bayer_pixel[0] - bayer_pixel[2] );
          if (dh > dv)
            gray_pixel[1] = AVG(bayer_pixel[1-line_skip], bayer_pixel[1+line_skip]);
          else if (dv > dh)
            gray_pixel[1] = AVG(bayer_pixel[0], bayer_pixel[2]);
          else
            gray_pixel[1] = AVG4(bayer_pixel[0], bayer_pixel[2], bayer_pixel[-line_skip+1], bayer_pixel[line_skip+1]);
        }
        gray_pixel[0] = bayer_pixel[0];
        gray_pixel[1] = AVG3(bayer_pixel[-line_skip+1], bayer_pixel[line_skip+1],bayer_pixel[-1]);
        gray_pixel += 2;
        bayer_pixel += 2;
      }

      // last line BGBGBG
      gray_pixel[0] = AVG(bayer_pixel[1], bayer_pixel[-line_skip]);
      gray_pixel[1] = bayer_pixel[1];
      gray_pixel += 2;
      bayer_pixel += 2;
      for (register unsigned xIdx = 2; xIdx < image.width; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
      {
        gray_pixel[0] = AVG3( bayer_pixel[-1], bayer_pixel[1], bayer_pixel[-line_skip] );
        gray_pixel[1] = bayer_pixel[1];
      }
    }
    else if (method == OpenNI_EdgeAwareWeighted)
    {
      int dv, dh;
      // first line GRGRGR
      for (register unsigned xIdx = 0; xIdx < image.width-2; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
      {
        gray_pixel[0] = bayer_pixel[0];            // green pixel
        gray_pixel[1] = AVG3(bayer_pixel[0], bayer_pixel[2], bayer_pixel[1+line_skip]); // interpolated green pixel
      }
      gray_pixel[0] = bayer_pixel[0];
      gray_pixel[1] = AVG( bayer_pixel[0], bayer_pixel[1+line_skip]);
      gray_pixel += 2;
      bayer_pixel += 2;

      for (register unsigned yIdx = 1; yIdx < image.height-1; yIdx += 2)
      {
        // blue line
        gray_pixel[0] = AVG3(bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[1]);
        gray_pixel[1] = bayer_pixel[1];
        gray_pixel += 2;
        bayer_pixel += 2;
        for (register unsigned xIdx = 2; xIdx < image.width; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
        {
          dv = abs( bayer_pixel[-line_skip] - bayer_pixel[line_skip] );
          dh = abs( bayer_pixel[-1] - bayer_pixel[1] );

          if (dv == 0 && dh == 0)
            gray_pixel[0] = AVG4(bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[-1], bayer_pixel[1]);
          else
            gray_pixel[0] = WAVG4(bayer_pixel[-line_skip], bayer_pixel[line_skip], bayer_pixel[-1], bayer_pixel[1], dh, dv);

          gray_pixel[1] = bayer_pixel[1];
        }

        // red line
        for (register unsigned xIdx = 0; xIdx < image.width-2; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
        {
          gray_pixel[0] = bayer_pixel[0];

          dv = abs( bayer_pixel[1-line_skip] - bayer_pixel[1+line_skip] );
          dh = abs( bayer_pixel[0] - bayer_pixel[2] );

          if (dv == 0 && dh == 0)
            gray_pixel[1] = AVG4(bayer_pixel[1-line_skip], bayer_pixel[1+line_skip], bayer_pixel[0], bayer_pixel[2]);
          else
            gray_pixel[1] = WAVG4(bayer_pixel[1-line_skip], bayer_pixel[1+line_skip], bayer_pixel[0], bayer_pixel[2], dh, dv);
        }
        gray_pixel[0] = bayer_pixel[0];
        gray_pixel[1] = AVG3(bayer_pixel[-line_skip+1], bayer_pixel[line_skip+1],bayer_pixel[-1]);
        gray_pixel += 2;
        bayer_pixel += 2;
      }

      // last line BGBGBG
      gray_pixel[0] = AVG(bayer_pixel[1], bayer_pixel[-line_skip]);
      gray_pixel[1] = bayer_pixel[1];
      gray_pixel += 2;
      bayer_pixel += 2;
      for (register unsigned xIdx = 2; xIdx < image.width; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
      {
        gray_pixel[0] = AVG3( bayer_pixel[-1], bayer_pixel[1], bayer_pixel[-line_skip] );
        gray_pixel[1] = bayer_pixel[1];
      }
    }
    else
    {
      ROS_WARN("[OpenNIDriver::Debayering] Unknown Debayering method %d", method );
    }

    // if (method)
  }
  else // downsampling
  {
    // fast method -> simply takes each or each 2nd pixel-group to get gray values out
    register unsigned bayer_step = bayer.XRes() / image.width;
    register unsigned bayer_skip = (bayer.YRes() / image.height - 1) * bayer.XRes();
    register const XnUInt8* bayer_buffer = bayer.Data();
    register unsigned char* gray_buffer = (unsigned char*)&image.data[0];

    for( register unsigned yIdx = 0; yIdx < bayer.YRes(); yIdx += bayer_step, bayer_buffer += bayer_skip ) // skip a line
    {
      for( register unsigned xIdx = 0; xIdx < bayer.XRes(); xIdx += bayer_step, ++gray_buffer, bayer_buffer += bayer_step )
      {
        *gray_buffer = AVG( bayer_buffer[0], bayer_buffer[ bayer.XRes() + 1]);
      }
    }
  } // downsampling
}

void OpenNIDriver::YUV2RGB ( const xn::ImageMetaData& yuv, sensor_msgs::Image& image ) const
{
  // u y1 v y2
  register const XnUInt8* yuv_buffer = yuv.Data();
  register unsigned char* rgb_buffer = (unsigned char*)&image.data[0];

  if (yuv.XRes() == image.width && yuv.YRes() == image.height)
  {
    for( register unsigned yIdx = 0; yIdx < yuv.YRes(); ++yIdx )
    {
      for( register unsigned xIdx = 0; xIdx < yuv.XRes(); xIdx += 2, rgb_buffer += 6, yuv_buffer += 4 )
      {
        rgb_buffer[0] = max (0, min (255, (int)yuv_buffer[1] + (int)(1.13983 * (float)(yuv_buffer[2]-128) ) ) );
        rgb_buffer[1] = max (0, min (255, (int)yuv_buffer[1] + (int)(124.832 -0.39465 * (float)yuv_buffer[0] - 0.58060 * (float)yuv_buffer[2] )));
        rgb_buffer[2] = max (0, min (255, (int)yuv_buffer[1] + (int)(2.03211 * (float)(yuv_buffer[0]-128)) ) );

        rgb_buffer[3] = max (0, min (255, (int)yuv_buffer[3] + (int)(1.13983 * (float)(yuv_buffer[2]-128) ) ) );
        rgb_buffer[4] = max (0, min (255, (int)yuv_buffer[3] + (int)(124.832 -0.39465 * (float)yuv_buffer[0] - 0.58060 * (float)yuv_buffer[2] )));
        rgb_buffer[5] = max (0, min (255, (int)yuv_buffer[3] + (int)(2.03211 * (float)(yuv_buffer[0]-128)) ) );
      }
    }
  }
  else
  {
    register unsigned yuv_step = yuv.XRes() / image.width;
    register unsigned yuv_x_step = yuv_step << 1;
    register unsigned yuv_skip = (yuv.YRes() / image.height - 1) * ( yuv.XRes() << 1 );
    for( register unsigned yIdx = 0; yIdx < yuv.YRes(); yIdx += yuv_step, yuv_buffer += yuv_skip )
    {
      for( register unsigned xIdx = 0; xIdx < yuv.XRes(); xIdx += yuv_step, rgb_buffer += 3, yuv_buffer += yuv_x_step )
      {
        rgb_buffer[0] = max (0, min (255, (int)yuv_buffer[1] + (int)(1.13983 * (float)(yuv_buffer[2]-128) ) ) );
        rgb_buffer[1] = max (0, min (255, (int)yuv_buffer[1] + (int)(124.832 -0.39465 * (float)yuv_buffer[0] - 0.58060 * (float)yuv_buffer[2] )));
        rgb_buffer[2] = max (0, min (255, (int)yuv_buffer[1] + (int)(2.03211 * (float)(yuv_buffer[0]-128)) ) );
      }
    }
  }
}

void OpenNIDriver::YUV2Gray ( const xn::ImageMetaData& yuv, sensor_msgs::Image& image ) const
{
  // u y1 v y2

  register unsigned yuv_step = yuv.XRes() / image.width;
  register unsigned yuv_x_step = yuv_step << 1;
  register unsigned yuv_skip = (yuv.YRes() / image.height - 1) * ( yuv.XRes() << 1 );
  register const XnUInt8* yuv_buffer = (yuv.Data() + 1);
  register unsigned char* gray_buffer = (unsigned char*)&image.data[0];

  for( register unsigned yIdx = 0; yIdx < yuv.YRes(); yIdx += yuv_step, yuv_buffer += yuv_skip )
  {
    for( register unsigned xIdx = 0; xIdx < yuv.XRes(); xIdx += yuv_step, ++gray_buffer, yuv_buffer += yuv_x_step )
    {
      *gray_buffer = *yuv_buffer;
    }
  }
}

} // namespace openni_camera
