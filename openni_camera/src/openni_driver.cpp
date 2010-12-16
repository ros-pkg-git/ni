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
#include <sensor_msgs/fill_image.h>
#include <boost/make_shared.hpp>
#include <vector>

// Branch on whether we have the changes to CameraInfo in unstable
#if ROS_VERSION_MINIMUM(1, 3, 0)
#include <sensor_msgs/distortion_models.h>
#endif

using namespace std;

namespace openni_camera 
{

const double OpenNIDriver::rgb_focal_length_ = 525;

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

/** \brief Constructor */
OpenNIDriver::OpenNIDriver (ros::NodeHandle comm_nh, ros::NodeHandle param_nh)
  : comm_nh_ (comm_nh)
  , param_nh_ (param_nh)
  , reconfigure_server_ (param_nh)
  , shadow_value_ (0)
  , no_sample_value_ (0)
{
  //cout << "OpenNIDriver::OpenNIDriver" << endl;
  // Init the OpenNI context
  XnStatus status = context_.Init ();

  if (status != XN_STATUS_OK)
  {
    ROS_ERROR ("[OpenNIDriver] Init: %s", xnGetStatusString (status));
    return;
  }
//  ROS_INFO ("[OpenNIDriver] Initialization successful.");

  // Set up reconfigure server
  ReconfigureServer::CallbackType f = boost::bind(&OpenNIDriver::configCb, this, _1, _2);
  reconfigure_server_.setCallback(f);
  
  // Set the frame IDs. The optical frames are all Z-forward.
  std::string openni_depth_frame, openni_RGB_frame;
  param_nh.param ("openni_depth_optical_frame", openni_depth_frame,
                  std::string ("/openni_depth_optical_frame"));
  param_nh.param ("openni_rgb_optical_frame", openni_RGB_frame,
                  std::string ("/openni_rgb_optical_frame"));
  
  cloud2_.header.frame_id = openni_depth_frame;  
  disp_image_.header.frame_id = openni_depth_frame;
  disp_image_.image.encoding = "32FC1";
  /// @todo IR (when we support it) is the same as the depth frame.  

  rgb_image_ .header.frame_id = openni_RGB_frame;
  gray_image_.header.frame_id = openni_RGB_frame;

  // Publishers and subscribers
  image_transport::ImageTransport it(comm_nh);
  pub_rgb_     = it.advertiseCamera ("rgb/image_color", 15); /// @todo 15 looks like overkill
  pub_gray_    = it.advertiseCamera ("rgb/image_mono", 15);
  pub_depth_image_ = it.advertise ("depth/image_raw", 15);
  pub_disparity_ = comm_nh_.advertise<stereo_msgs::DisparityImage>("depth/disparity", 15 );
  pub_depth_points2_ = comm_nh.advertise<sensor_msgs::PointCloud2>("depth/points2", 15);

  SyncPolicy sync_policy(4); // queue size
  /// @todo Set inter-message lower bound, age penalty, max interval to lower latency
  // Connect no inputs, we'll add messages manually
  depth_rgb_sync_.reset( new Synchronizer(sync_policy) );
  depth_rgb_sync_->registerCallback(boost::bind(&OpenNIDriver::publishRegisteredPointCloud,
                                                this, _1, _2));

  //cout << "OpenNIDriver::OpenNIDriver...end" << endl;
}

/** \brief Destructor */
OpenNIDriver::~OpenNIDriver ()
{
  context_.StopGeneratingAll ();
  context_.Shutdown ();
}

bool OpenNIDriver::isRGBRequired() const
{
  return ( ( pub_rgb_.getNumSubscribers() > 0 ) ||
           ( pub_depth_points2_.getNumSubscribers() > 0 && config_.point_cloud_type == OpenNI_XYZRGB )
         );
}

bool OpenNIDriver::isGrayRequired() const
{
  return ( ( pub_gray_.getNumSubscribers() > 0 ) );
}

bool OpenNIDriver::isImageStreamRequired() const
{
  return ( isRGBRequired() || isGrayRequired() );
}

bool OpenNIDriver::isDepthStreamRequired() const
{
  return ( pub_depth_points2_.getNumSubscribers() > 0 ||
           pub_disparity_.getNumSubscribers() > 0     ||
           pub_depth_image_.getNumSubscribers() > 0 );
}

/** \brief Spin loop. */
bool 
OpenNIDriver::spin ()
{
  /// @Pat: I've disabled this code from publish() now for simplicity. I suspect that
  /// with IsNewDataAvailable (before updating) we can get more accurate timestamp offsets.
  /*
  static bool first_publish = true;
  static ros::Duration time_offset;
  static XnUInt64 last_image_timestamp = 0;
  static XnUInt64 last_depth_timestamp = 0;

  if (first_publish)
  {
    last_image_timestamp = image_generator_.GetTimestamp ();
    last_depth_timestamp = depth_generator_.GetTimestamp ();
    ros::Time ros_time = ros::Time::now ();

    XnUInt64 first_image_timestamp = min( last_image_timestamp, last_depth_timestamp );

    ros::Time current (first_image_timestamp / 1000000, (first_image_timestamp % 1000000) * 1000);

    time_offset = ros_time - current;

    first_publish = false;
    return; // dont publish right now!
  }

  XnUInt64 image_timestamp = image_generator_.GetTimestamp ();
  XnUInt64 depth_timestamp = depth_generator_.GetTimestamp ();

  if (image_timestamp != last_image_timestamp)
  {
    ros::Time time (image_timestamp / 1000000, (image_timestamp % 1000000) * 1000);
    time += time_offset;
    // etc.
  }
  */
  
  XnStatus status;
  //cout << "OpenNIDriver::spin" << endl;
  ros::Duration r (0.01);

  while (comm_nh_.ok ())
  {
    if (!isImageStreamRequired() && image_generator_.IsGenerating())
    {
      //cout << "stopping image stream..." << flush;
      status = image_generator_.StopGenerating();
      if (status != XN_STATUS_OK)
      {
        ROS_ERROR ("[OpenNIDriver::spin] Error in stopping image stream (): %s", xnGetStatusString (status));
        return (false);
      }
      //cout << "OK" << endl;
    }
    else if (isImageStreamRequired() && !image_generator_.IsGenerating())
    {
      //cout << "starting image stream..." << flush;
      status = image_generator_.StartGenerating();
      if (status != XN_STATUS_OK)
      {
        ROS_ERROR ("[OpenNIDriver::spin] Error in starting image stream (): %s", xnGetStatusString (status));
        return (false);
      }
      //cout << "OK" << endl;
    }

    if (!isDepthStreamRequired() && depth_generator_.IsGenerating())
    {
      //cout << "stopping depth stream..." << flush;
      status = depth_generator_.StopGenerating();
      if (status != XN_STATUS_OK)
      {
        ROS_ERROR ("[OpenNIDriver::spin] Error in stopping depth stream (): %s", xnGetStatusString (status));
        return (false);
      }
      //cout << "OK" << endl;
    }
    else if (isDepthStreamRequired() && !depth_generator_.IsGenerating())
    {
      //cout << "starting depth stream..." << flush;
      status = depth_generator_.StartGenerating();
      if (status != XN_STATUS_OK)
      {
        ROS_ERROR ("[OpenNIDriver::spin] Error in starting depth stream (): %s", xnGetStatusString (status));
        return (false);
      }
      //cout << "OK" << endl;

      if (config_.point_cloud_type != OpenNI_XYZ_unregistered)
      {
        //cout << "switching on registration..." << flush;
        status = depth_generator_.GetAlternativeViewPointCap().SetViewPoint( image_generator_ );
        if (status != XN_STATUS_OK)
        {
          ROS_ERROR ("[OpenNIDriver::spin] Error in switching on depth stream registration: %s", xnGetStatusString (status));
          return (false);
        }
        //cout << "OK" << endl;
      }
      else
      {
        //cout << "switching off registration..." << flush;
        status = depth_generator_.GetAlternativeViewPointCap().ResetViewPoint();
        if (status != XN_STATUS_OK)
        {
          ROS_ERROR ("[OpenNIDriver::spin] Error in switching off depth stream registration: %s", xnGetStatusString (status));
          return (false);
        }
        //cout << "OK" << endl;
      }
    }

    if (!isImageStreamRequired() && !isDepthStreamRequired())
    {
      //cout << "no subscribers -> sleep..." << endl;
      // wait for subscribers!
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

    // Spin for ROS message processing
    ros::spinOnce ();
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
  sensor_msgs::ImagePtr depth_ptr;
  if (pub_depth_image_.getNumSubscribers () > 0 ||
      (pub_depth_points2_.getNumSubscribers() > 0 && config_.point_cloud_type == OpenNI_XYZRGB))
  {
    depth_ptr = boost::make_shared<sensor_msgs::Image>();
    depth_ptr->header.stamp = time;
    depth_ptr->header.frame_id = disp_image_.header.frame_id;
    sensor_msgs::fillImage(*depth_ptr, sensor_msgs::image_encodings::TYPE_16UC1,
                           depth_md.YRes(), depth_md.XRes(), depth_md.XRes() * sizeof(uint16_t),
                           (void*)depth_md.Data());
    pub_depth_image_.publish(depth_ptr);
  }

  // Disparity image
  if (pub_disparity_.getNumSubscribers () > 0)
  {
    disp_image_.header.stamp = time;
    publishDisparity(depth_md);
  }

  // Point cloud
  if (pub_depth_points2_.getNumSubscribers() > 0 )
  {
    cloud2_.header.stamp = time;

    if (config_.point_cloud_type == OpenNI_XYZRGB)
      depth_rgb_sync_->add<0>(depth_ptr);
    else
      publishUnregisteredPointCloud(depth_md);
  }
}

void OpenNIDriver::processRgb ()
{
  /// @todo Some sort of offset based on the hardware time
  ros::Time time = ros::Time::now();
  
  xn::ImageMetaData image_md;
  image_generator_.GetMetaData( image_md );

  if (isRGBRequired())
  {
    rgb_image_.header.stamp = rgb_info_.header.stamp = time;

    bayer2RGB( image_md, rgb_image_, config_.Bayer2RGB );

    sensor_msgs::ImageConstPtr rgb_ptr = boost::make_shared<const sensor_msgs::Image> (rgb_image_);
    if (pub_rgb_.getNumSubscribers() > 0)
    {
      pub_rgb_.publish (rgb_ptr,
                        boost::make_shared<const sensor_msgs::CameraInfo> (rgb_info_));
    }

    if (pub_depth_points2_.getNumSubscribers() > 0 && config_.point_cloud_type == OpenNI_XYZRGB)
      depth_rgb_sync_->add<1>(rgb_ptr);
  }

  if (isGrayRequired())
  {
    gray_image_.header.stamp = rgb_info_.header.stamp = time;
    gray_image_.header.seq = image_generator_.GetFrameID ();

    bayer2Gray( image_md, gray_image_, config_.Bayer2RGB );

    if (pub_gray_.getNumSubscribers() > 0)
    {
      pub_gray_.publish (boost::make_shared<const sensor_msgs::Image> (gray_image_),
                         boost::make_shared<const sensor_msgs::CameraInfo> (rgb_info_));
    }
  }
}

void OpenNIDriver::publishDisparity ( const xn::DepthMetaData& depth_md )
{
  unsigned xStep = 640 / disp_image_.image.width;
  unsigned ySkip = (480 / disp_image_.image.height - 1) * 640;

  // Fill in the depth image data
  // iterate over all elements and fill disparity matrix: disp[x,y] = f * b / z_distance[x,y];
  float constant = focal_length_ * baseline_ * 1000.0;
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
  pub_disparity_.publish ( disp_image_ );
}

void OpenNIDriver::publishUnregisteredPointCloud ( const xn::DepthMetaData& depth_md )
{
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  int depth_idx = 0;
  float* pt_data = reinterpret_cast<float*>(&cloud2_.data[0]);
  float constant = pixel_size_ * 0.001 / F_;

  unsigned depthStep = depth_md.XRes () / cloud2_.width;
  unsigned depthSkip = (depth_md.YRes () / cloud2_.height - 1) * depth_md.XRes ();

  int centerX = cloud2_.width >> 1;
  int centerY = cloud2_.height >> 1;
  constant *= depthStep;

  for (int v = 0; v < (int)cloud2_.height; ++v, depth_idx += depthSkip)
  {
    for (int u = 0; u < (int)cloud2_.width; ++u, depth_idx += depthStep, pt_data += cloud2_.fields.size())
    {
      // Check for invalid measurements
      if (depth_md[depth_idx] == 0 ||
          depth_md[depth_idx] == no_sample_value_ ||
          depth_md[depth_idx] == shadow_value_)
      {
        // not valid
        pt_data[0] = bad_point;
        pt_data[1] = bad_point;
        pt_data[2] = bad_point;
        continue;
      }

      // Fill in XYZ
      pt_data[0] = (u - centerX) * depth_md[depth_idx] * constant;
      pt_data[1] = (v - centerY) * depth_md[depth_idx] * constant;
      pt_data[2] = depth_md[depth_idx] * 0.001;
    }
  }

  /// @todo Depth frame here
  pub_depth_points2_.publish (boost::make_shared<const sensor_msgs::PointCloud2> (cloud2_));
}

void OpenNIDriver::publishRegisteredPointCloud ( const sensor_msgs::ImageConstPtr& depth_msg,
                                                 const sensor_msgs::ImageConstPtr& rgb_msg )
{
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  int depth_idx = 0;
  float* pt_data = reinterpret_cast<float*>(&cloud2_.data[0]);
  float constant = pixel_size_ * 0.001 / F_;

  //unsigned depthStep = depth_md.XRes () / cloud2_.width;
  //unsigned depthSkip = (depth_md.YRes () / cloud2_.height - 1) * depth_md.XRes ();
  unsigned depthStep = depth_msg->width / cloud2_.width;
  unsigned depthSkip = (depth_msg->height / cloud2_.height - 1) * depth_msg->width;

  int centerX = cloud2_.width >> 1;
  int centerY = cloud2_.height >> 1;
  constant *= depthStep;

  unsigned char* rgb_buffer = (unsigned char*)&rgb_image_.data[0];
  RGBValue color;
  color.Alpha = 0;

  int color_idx = 0;
  unsigned colorStep = 3 * rgb_image_.width / cloud2_.width;
  unsigned colorSkip = 3 * (rgb_image_.height / cloud2_.height - 1) * rgb_image_.width;

  /// @todo Right now we always copy the raw depth data into a sensor_msgs/Image. In principle this
  /// isn't completely avoidable, as the synchronizer may wait for the next depth image before choosing
  /// the old data, which is overwritten by that point. In practice though we should optimize to avoid
  /// this almost always.
  const uint16_t* depth_md = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);

  for (int v = 0; v < (int)cloud2_.height; ++v, depth_idx += depthSkip, color_idx += colorSkip)
  {
    for (int u = 0; u < (int)cloud2_.width; ++u, depth_idx += depthStep, pt_data += cloud2_.fields.size(), color_idx += colorStep)
    {
      // Check for invalid measurements
      if (depth_md[depth_idx] == 0 ||
          depth_md[depth_idx] == no_sample_value_ ||
          depth_md[depth_idx] == shadow_value_)
      {
        // not valid
        pt_data[0] = bad_point;
        pt_data[1] = bad_point;
        pt_data[2] = bad_point;
        pt_data[3] = bad_point;
        continue;
      }

      // Fill in XYZ
      pt_data[0] = (u - centerX) * depth_md[depth_idx] * constant;
      pt_data[1] = (v - centerY) * depth_md[depth_idx] * constant;
      pt_data[2] = depth_md[depth_idx] * 0.001;

      // Fill in color
      color.Red   = rgb_buffer[color_idx];
      color.Green = rgb_buffer[color_idx + 1];
      color.Blue  = rgb_buffer[color_idx + 2];
      pt_data[3] = color.float_value;
    }
  }

  /// @todo RGB frame here
  pub_depth_points2_.publish (boost::make_shared<const sensor_msgs::PointCloud2> (cloud2_));
}

void OpenNIDriver::configCb (Config &config, uint32_t level)
{
  // if our pointcloud is VGA and XYZRGB -> we need at least VGA resolution for image
  if( config.point_cloud_resolution > config.image_resolution && config.point_cloud_type )
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

  switch( config_.image_resolution )
  {
    case OpenNI_QQVGA_30Hz:
            stream_width   = 640;
            stream_height  = 480;
            stream_fps     = 30;
            image_width    = 160;
            image_height   = 120;
      break;

    case OpenNI_QVGA_30Hz:
            stream_width   = 640;
            stream_height  = 480;
            stream_fps     = 30;
            image_width    = 320;
            image_height   = 240;
      break;

    case OpenNI_VGA_30Hz:
            stream_width   = 640;
            stream_height  = 480;
            stream_fps     = 30;
            image_width    = 640;
            image_height   = 480;
      break;

    case OpenNI_SXGA_15Hz:
            stream_width   = 1280;
            stream_height  = 1024;
            stream_fps     = 15;
            image_width    = 1280;
            image_height   = 1024;
      break;

    default:ROS_WARN("Unknwon image resolution - falling back to VGA@30Hz");
            stream_width   = 640;
            stream_height  = 480;
            stream_fps     = 30;
            image_width    = 640;
            image_height   = 480;
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
            cloud2_.height = 480;
            cloud2_.width  = 640;
            break;
            
    default: ROS_WARN("Unknwon point cloud size - falling back to VGA");
            cloud2_.height = 480;
            cloud2_.width  = 640;
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
        disp_image_.image.height = 480;
        disp_image_.image.width = 640;
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
      ROS_ERROR ("[OpenNIDriver] Failed to create DepthGenerator: %s", xnGetStatusString (rc_));
      return (false);
    }

    // Set the correct mode on the depth/image generator
    mode.nXRes = 640;
    mode.nYRes = 480;
    mode.nFPS  = 30;
    if (depth_generator_.SetMapOutputMode (mode) != XN_STATUS_OK)
    {
      ROS_ERROR("[OpenNIDriver] Failed to set depth output mode");
      return (false);
    }

    // Read parameters from the camera
    if (depth_generator_.GetRealProperty ("ZPPS", pixel_size_) != XN_STATUS_OK)
      ROS_ERROR ("[OpenNIDriver] Could not read pixel size!");
//    else
//      ROS_INFO_STREAM ("[OpenNIDriver] Pixel size: " << pixel_size_);

    pixel_size_ *= 2.0;

    if (depth_generator_.GetIntProperty ("ZPD", F_) != XN_STATUS_OK)
      ROS_ERROR ("[OpenNIDriver] Could not read virtual plane distance!");
//    else
//      ROS_INFO_STREAM ("[OpenNIDriver] Virtual plane distance: " << F_);

    if (depth_generator_.GetRealProperty ("LDDIS", baseline_) != XN_STATUS_OK)
      ROS_ERROR ("[OpenNIDriver] Could not read base line!");
//    else
//      ROS_INFO_STREAM ("[OpenNIDriver] Base line: " << baseline_);

    // baseline from cm -> meters
    baseline_ *= 0.01;

    //focal length from mm -> pixels (valid for 640x480)
    focal_length_ = (double)F_/pixel_size_;

    if (depth_generator_.GetIntProperty ("ShadowValue", shadow_value_) != XN_STATUS_OK)
      ROS_WARN ("[OpenNIDriver] Could not read shadow value!");

    if (depth_generator_.GetIntProperty ("NoSampleValue", no_sample_value_) != XN_STATUS_OK)
      ROS_WARN ("[OpenNIDriver] Could not read no sample value!");

  }

  // got baseline update disparity image
  disp_image_.T = baseline_;
  disp_image_.f  = focal_length_;
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

  rgb_info_.K[0] = rgb_info_.K[4] = rgb_focal_length_ * (double)image_width / 640.0;
  rgb_info_.K[2] = image_width >> 1;
  rgb_info_.K[5] = image_height >> 1;
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

  // InputFormat should be 6 for Kinect, 5 for PS
  int image_input_format = 6;
  if (param_nh_.getParam ("image_input_format", image_input_format))
  {
    if (image_generator_.SetIntProperty ("InputFormat", image_input_format) != XN_STATUS_OK)
      ROS_ERROR ("[OpenNIDriver] Error setting the image input format to Uncompressed 8-bit BAYER!");
  }
  
  // RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS
  int registration_type = 0;
  if (param_nh_.getParam ("registration_type", registration_type))
  {
    if (depth_generator_.SetIntProperty ("RegistrationType", registration_type) != XN_STATUS_OK)
      ROS_WARN ("[OpenNIDriver] Error enabling registration!");
  }

  if (image_generator_.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT )!= XN_STATUS_OK)
  {
    ROS_ERROR("[OpenNIDriver] Failed to set image pixel format");
    return (false);
  }

  if (config_.point_cloud_type == OpenNI_XYZ_unregistered) // not registered pc
  {
    //cout << "switching off registration..." << flush;
    status = depth_generator_.GetAlternativeViewPointCap().ResetViewPoint();
    if (status != XN_STATUS_OK)
    {
      ROS_ERROR ("[OpenNIDriver::spin] Error in switching off registering on depth stream: %s", xnGetStatusString (status));
      return (false);
    }
    //cout << "OK" << endl;
  }
  else
  {
    //cout << "switching on registration..." << flush;
    status = depth_generator_.GetAlternativeViewPointCap().SetViewPoint( image_generator_ );
    if (status != XN_STATUS_OK)
    {
      ROS_ERROR ("[OpenNIDriver::spin] Error in switching on registering on depth stream: %s", xnGetStatusString (status));
      return (false);
    }
    //cout << "OK" << endl;
  }

  return true;
}

void OpenNIDriver::bayer2RGB ( const xn::ImageMetaData& bayer, sensor_msgs::Image& image, int method )
{
  if (bayer.XRes() == image.width && bayer.YRes() == image.height)
  {
    const XnUInt8 *bayer_pixel = bayer.Data();
    unsigned yIdx, xIdx;
    XnUInt8 v;

    int line_step = image.width;
    int line_step2 = image.width << 1;

    int rgb_line_step2 = line_step2 * 3;          // previous 2 color lines
    int rgb_line_step  = line_step * 3;             // previous color line
    unsigned char *rgb_pixel = (unsigned char *)&image.data[0];

    if (method == 0)
    {
      for (yIdx = 0; yIdx < image.height; yIdx += 2)
      {
        // red line (GRGR...)
        for (xIdx=0; xIdx < image.width; xIdx += 2, rgb_pixel += 6)
        {
          *( rgb_pixel + 1 ) = *bayer_pixel++;   // green pixel
          v = *bayer_pixel++;
          *( rgb_pixel + 3 ) = v;    // red pixel
          *( rgb_pixel ) = AVG(*(rgb_pixel + 3), *(rgb_pixel - 3)); // interpolated red pixel
          if (yIdx > 1)
          {
            *( rgb_pixel - rgb_line_step) = AVG(*(rgb_pixel-rgb_line_step2+0), *(rgb_pixel+0)); // interpolated red pixel
            *( rgb_pixel - rgb_line_step + 3 ) = AVG(*(rgb_pixel-rgb_line_step2+3+0), *(rgb_pixel+3+0)); // interpolated red pixel
            *( rgb_pixel - rgb_line_step + 1 ) = ((int)*(rgb_pixel+1) + (int)*(rgb_pixel-rgb_line_step-3+1) + (int)*(rgb_pixel-rgb_line_step+3+1) + (int)*(rgb_pixel-rgb_line_step2+1)) >> 2;
          }
        }

        // blue line (BGBG...)
        v = *bayer_pixel;
        *(rgb_pixel+2) = v;     // blue pixel
        for ( xIdx = 0; xIdx < image.width - 2; xIdx+=2, rgb_pixel += 6 )
        {
          bayer_pixel++;
          //              *(dd+1) =
          *(rgb_pixel+3+1) = *bayer_pixel++;      // green pixel
          v = *bayer_pixel;
          *(rgb_pixel+6+2) = v;
          *(rgb_pixel+3+2) = AVG(*(rgb_pixel+2), *(rgb_pixel+6+2)); // interpolated blue pixel
          if (yIdx > 1)
          {
            *(rgb_pixel-rgb_line_step+2) = AVG(*(rgb_pixel-rgb_line_step2+2), *(rgb_pixel+2)); // interpolated blue pixel
            *(rgb_pixel-rgb_line_step+3+2) = AVG(*(rgb_pixel-rgb_line_step2+3+2), *(rgb_pixel+3+2)); // interpolated blue pixel
            //                  *(dd-image.width+1) =
            *(rgb_pixel-rgb_line_step+3+1) = ((int)*(rgb_pixel+3+1) + (int)*(rgb_pixel-rgb_line_step+1) + (int)*(rgb_pixel-rgb_line_step+6+1) + (int)*(rgb_pixel-rgb_line_step2+3+1)) >> 2;
          }
        }
        // last pixels
        bayer_pixel++;
        *(rgb_pixel+3+1) = *bayer_pixel++;      // green pixel
        *(rgb_pixel+3+2) = *(rgb_pixel+2);	// interpolated blue pixel
        if (yIdx > 1)
        {
          *(rgb_pixel-rgb_line_step+2) = AVG(*(rgb_pixel-rgb_line_step2+2), *(rgb_pixel+2)); // interpolated blue pixel
          *(rgb_pixel-rgb_line_step+3+2) = AVG(*(rgb_pixel-rgb_line_step2+3+2), *(rgb_pixel+3+2)); // interpolated blue pixel
        }
        rgb_pixel += 6;
      }
    }
    else if (method == 1)
    {
      int a,b,c,d;
      int dc, dv, dh;
      int ww;

      // do first two lines
      rgb_pixel += rgb_line_step2;
      //      dd += line_step2;
      bayer_pixel += line_step2;

      for (yIdx=0; yIdx<image.height-4; yIdx+=2)
        {
          // GR line
          // do first two pixels
          rgb_pixel += 6;
          //          dd += 2;
          bayer_pixel += 2;

          // do most of line
          for (xIdx=0; xIdx<image.width-4; xIdx+=2, rgb_pixel+=6)
            {
              // green pixels
              //              *dd++ =
              *(rgb_pixel+1) = *bayer_pixel++;
              dc = 2*(int)*(bayer_pixel);
              dh = dc - (int)*(bayer_pixel-2) - (int)*(bayer_pixel+2);
              if (dh < 0) dh = -dh;
              dv = dc - (int)*(bayer_pixel-line_step2) - (int)*(bayer_pixel+line_step2);
              if (dv < 0) dv = -dv;
              if (dv > dh) // vert is stronger, use horz
                //*dd++ =
                    *(rgb_pixel+3+1) = ((int)*(bayer_pixel-1) + (int)*(bayer_pixel+1))>>1;
              else	// horz is stronger, use vert
                //*dd++ =
                    *(rgb_pixel+3+1) = ((int)*(bayer_pixel-line_step) + (int)*(bayer_pixel+line_step))>>1;

              // color pixels
              *(rgb_pixel+3+0) = *bayer_pixel;	// red pixel

              a = (int)*(bayer_pixel) - (int)*(rgb_pixel+3+1);
              b = (int)*(bayer_pixel-2) - (int)*(rgb_pixel-3+1);
              c = (int)*(bayer_pixel-line_step2) - (int)*(rgb_pixel-rgb_line_step2+3+1);
              d = (int)*(bayer_pixel-line_step2-2) - (int)*(rgb_pixel-rgb_line_step2-3+1);

              ww = 2*(int)*(rgb_pixel+1) + (a + b);
              if (ww < 0) ww = 0;
              if (ww > 511) ww = 511;
              *(rgb_pixel+0) = ww>>1;	// interpolated red pixel

              ww = 2*(int)*(rgb_pixel-rgb_line_step+3+1) + (a + c);
              if (ww < 0) ww = 0;
              if (ww > 511) ww = 511;
              *(rgb_pixel-rgb_line_step+3+0) = ww>>1; // interpolated red pixel

              ww = 4*(int)*(rgb_pixel-rgb_line_step+1) + (a + b + c + d);
              if (ww < 0) ww = 0;
              if (ww > 1023) ww = 1023;
              *(rgb_pixel-rgb_line_step+0) = ww>>2; // interpolated red pixel

              bayer_pixel++;
            }
          // last two pixels
          rgb_pixel += 6;
          //          dd += 2;
          bayer_pixel += 2;

          // BG line
          // do first two pixels
          rgb_pixel += 6;
          //          dd += 2;
          bayer_pixel += 2;

          // do most of line
          for (xIdx=0; xIdx<image.width-4; xIdx+=2, rgb_pixel+=6)
            {
              dc = 2*(int)*bayer_pixel;
              dh = dc - (int)*(bayer_pixel-2) - (int)*(bayer_pixel+2);
              if (dh < 0) dh = -dh;
              dv = dc - (int)*(bayer_pixel-line_step2) - (int)*(bayer_pixel+line_step2);
              if (dv < 0) dv = -dv;
              if (dh < dv) // vert is stronger, use horz
                //*dd++ =
                    *(rgb_pixel+1) = ((int)*(bayer_pixel-1) + (int)*(bayer_pixel+1))>>1;
              else	// horz is stronger, use vert
                //*dd++ =
                    *(rgb_pixel+1) = ((int)*(bayer_pixel-line_step) + (int)*(bayer_pixel+line_step))>>1;
              //              *dd++ =
              *(rgb_pixel+3+1) = *(bayer_pixel+1); // green pixel

              // color pixels
              *(rgb_pixel+3+2) = *bayer_pixel;	// blue pixel

              a = (int)*(bayer_pixel) - (int)*(rgb_pixel+3+1);
              b = (int)*(bayer_pixel-2) - (int)*(rgb_pixel-3+1);
              c = (int)*(bayer_pixel-line_step2) - (int)*(rgb_pixel-rgb_line_step2+3+1);
              d = (int)*(bayer_pixel-line_step2-2) - (int)*(rgb_pixel-rgb_line_step2-3+1);

              ww = 2*(int)*(rgb_pixel+1) + (a + b);
              if (ww < 0) ww = 0;
              if (ww > 511) ww = 511;
              *(rgb_pixel+2) = ww>>1;	// interpolated blue pixel

              ww = 2*(int)*(rgb_pixel-rgb_line_step+3+1) + (a + c);
              if (ww < 0) ww = 0;
              if (ww > 511) ww = 511;
              *(rgb_pixel-rgb_line_step+3+2) = ww>>1; // interpolated blue pixel

              ww = 4*(int)*(rgb_pixel-rgb_line_step+1) + (a + b + c + d);
              if (ww < 0) ww = 0;
              if (ww > 1023) ww = 1023;
              *(rgb_pixel-rgb_line_step+2) = ww>>2; // interpolated blue pixel

              bayer_pixel+=2;
            }
          // last two pixels
          rgb_pixel += 6;
          //          dd += 2;
          bayer_pixel += 2;
        }
    }
    else
    {
      ROS_WARN("[OpenNIDriver::bayer2RGB] Unknown Bayer2RGB conversion method %d", method );
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

void OpenNIDriver::bayer2Gray ( const xn::ImageMetaData& bayer, sensor_msgs::Image& image, int method )
{
  if (bayer.XRes() == image.width && bayer.YRes() == image.height)
  {
    unsigned char* gray_pixel = (unsigned char *)&image.data[0];
    const XnUInt8 *bayer_pixel = bayer.Data();
    int line_skip = image.width;
    int line_skip2 = image.width << 1;

    if (method == 0)
    {
      for (register unsigned yIdx = 0; yIdx < image.height; yIdx += 2)
      {
        // red line
        for (register unsigned xIdx = 0; xIdx < image.width-2; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
        {
          gray_pixel[0] = bayer_pixel[0];            // green pixel
          gray_pixel[1] = AVG(gray_pixel[0], bayer_pixel[2]); // interpolated green pixel
        }
        gray_pixel[0] = gray_pixel[1] = bayer_pixel[0];
        gray_pixel += 2;
        bayer_pixel += 2;


        // blue line
        gray_pixel[0] = gray_pixel[1] = bayer_pixel[1];
        gray_pixel += 2;
        bayer_pixel += 2;
        for (register unsigned xIdx = 2; xIdx < image.width; xIdx += 2, gray_pixel += 2, bayer_pixel += 2)
        {
          gray_pixel[1] = bayer_pixel[1];        // green pixel
          gray_pixel[0] = AVG(gray_pixel[1], gray_pixel[-1]); // interpolated green pixel
        }
      }

    }
    else // best
    {
      int dc, dv, dh;

      // do first two lines

      // GRGRGR line
      for (register unsigned xIdx = 0; xIdx < image.width - 2; xIdx += 2, gray_pixel +=2, bayer_pixel += 2)
      {
        // for first line we have GRGRGR -> interpolate between left and right gren value
        gray_pixel[0] = bayer_pixel[0];
        gray_pixel[1] = ((int)bayer_pixel[0] + (int)bayer_pixel[2]) >> 1;
      }
      // last two pixels of the GRGR line
      gray_pixel[0] = gray_pixel[1] = bayer_pixel[0];
      
      gray_pixel += 4;
      bayer_pixel += 4;
      // first two pixels of BGBGBG line
      gray_pixel[-2] = gray_pixel[-1] = bayer_pixel[-1];
      for (register unsigned xIdx = 2; xIdx < image.width; xIdx += 2, gray_pixel +=2, bayer_pixel += 2)
      {
        // for first line we have GRGRGR -> interpolate between left and right gren value
        gray_pixel[1] = bayer_pixel[1];
        gray_pixel[0] = ((int)bayer_pixel[-1] + (int)bayer_pixel[1]) >> 1;
      }

      //gray_pixel += line_skip2;
      //bayer_pixel += line_skip2;

      for (register unsigned yIdx = 0; yIdx < image.height - 4; yIdx += 2)
      {
        // GR line
        // do first two pixels
        gray_pixel[0] = bayer_pixel[0];
        // simply use vertical interpolation
        gray_pixel[1] = ((int)bayer_pixel[1-line_skip] + (int)bayer_pixel[1+line_skip]) >> 1;
        gray_pixel += 2;
        bayer_pixel += 2;

        // do most of line
        for (register unsigned xIdx = 0; xIdx < image.width - 4; xIdx += 2, gray_pixel +=2, bayer_pixel += 2)
        {
          // green pixel
          gray_pixel[0] = bayer_pixel[0];

          // red pixel
          // center
          dc = ((int)bayer_pixel[1]) << 1;
          // horizontal gradient on red pixels
          dh = dc - ((int)bayer_pixel[-1])- (int)bayer_pixel[3];
          if (dh < 0) dh = -dh;
          // vertical gradient on red pixels
          dv = dc - (int)bayer_pixel[1-line_skip2] - (int)bayer_pixel[1+line_skip2];
          if (dv < 0) dv = -dv;
          if (dv > dh) // vert is stronger, use horz
            gray_pixel[1] = ((int)bayer_pixel[0] + (int)bayer_pixel[2]) >> 1;
          else  // horz is stronger, use vert
            gray_pixel[1] = ((int)bayer_pixel[1-line_skip] + (int)bayer_pixel[1+line_skip]) >> 1;
        }
        gray_pixel[0] = bayer_pixel[0];
        // use vertical interpolation, since horizontal is not available here
        gray_pixel[1] = ((int)bayer_pixel[1-line_skip] + (int)bayer_pixel[1+line_skip]) >> 1;

        // last two pixels
        gray_pixel += 2;
        bayer_pixel += 2;

        // BG line
        // do first two pixels

        gray_pixel[1] = bayer_pixel[1];
        // simply use vertical interpolation
        gray_pixel[0] = ((int)bayer_pixel[line_skip] + (int)bayer_pixel[line_skip]) >> 1;
        gray_pixel += 2;
        bayer_pixel += 2;

        // do most of line
        for (register unsigned xIdx = 0; xIdx < image.width - 4; xIdx += 2, bayer_pixel += 2, gray_pixel += 2)
        {
          dc = ((int)bayer_pixel[0]) << 1;

          dh = dc - (int)bayer_pixel[-2] - (int)bayer_pixel[2];
          if (dh < 0) dh = -dh;

          dv = dc - (int)bayer_pixel[-line_skip2] - (int)bayer_pixel[line_skip2];
          if (dv < 0) dv = -dv;
          if (dh < dv) // vert is stronger, use horz
            gray_pixel[0] = ((int)bayer_pixel[-1] + (int)bayer_pixel[1]) >> 1;
          else  // horz is stronger, use vert
            gray_pixel[0] = ((int)bayer_pixel[-line_skip] + (int)bayer_pixel[line_skip]) >> 1;
          gray_pixel[1] = bayer_pixel[1]; // green pixel

        }

        // last two pixels
        gray_pixel[1] = bayer_pixel[1];
        // simply use vertical interpolation
        gray_pixel[0] = ((int)bayer_pixel[line_skip] + (int)bayer_pixel[line_skip]) >> 1;
        
        gray_pixel += 2;
        bayer_pixel += 2;
      }

      // do last two lines

      // GRGRGR line
      for (register unsigned xIdx = 0; xIdx < image.width - 2; xIdx += 2, gray_pixel +=2, bayer_pixel += 2)
      {
        // for first line we have GRGRGR -> interpolate between left and right gren value
        gray_pixel[0] = bayer_pixel[0];
        gray_pixel[1] = ((int)bayer_pixel[0] + (int)bayer_pixel[2]) >> 1;
      }
      // last two pixels of the GRGR line
      gray_pixel[0] = gray_pixel[1] = bayer_pixel[0];

      gray_pixel += 4;
      bayer_pixel += 4;
      // first two pixels of BGBGBG line
      gray_pixel[-2] = gray_pixel[-1] = bayer_pixel[-1];
      for (register unsigned xIdx = 2; xIdx < image.width; xIdx += 2, gray_pixel +=2, bayer_pixel += 2)
      {
        // for first line we have GRGRGR -> interpolate between left and right gren value
        gray_pixel[1] = bayer_pixel[1];
        gray_pixel[0] = ((int)bayer_pixel[-1] + (int)bayer_pixel[1]) >> 1;
      }
    } // if (method)
  }
  else // downsampling
  {

    // fast method -> simply takes each or each 2nd pixel-group to get gray values out
    register unsigned bayer_step = bayer.XRes() / image.width;
    register unsigned bayer_skip = (bayer.YRes() / image.height - 1) * bayer.XRes();
    register const XnUInt8* bayer_buffer = bayer.Data();
    register unsigned char* gray_buffer = (unsigned char*)&image.data[0];
    if (method==0)
    {
      for( register unsigned yIdx = 0; yIdx < bayer.YRes(); yIdx += bayer_step, bayer_buffer += bayer_skip ) // skip a line
      {
        for( register unsigned xIdx = 0; xIdx < bayer.XRes(); xIdx += bayer_step, ++gray_buffer, bayer_buffer += bayer_step )
        {
          *gray_buffer = AVG4( bayer_buffer[ bayer.XRes() ], bayer_buffer[0], bayer_buffer[ bayer.XRes() + 1], bayer_buffer[ 1 ]);
        }
      }
    }
    else
    {
      for( register unsigned yIdx = 0; yIdx < bayer.YRes(); yIdx += bayer_step, bayer_buffer += bayer_skip ) // skip a line
      {
        for( register unsigned xIdx = 0; xIdx < bayer.XRes(); xIdx += bayer_step, ++gray_buffer, bayer_buffer += bayer_step )
        {
          *gray_buffer = AVG( bayer_buffer[0], bayer_buffer[ bayer.XRes() + 1]);
        }
      }
    }
  } // downsampling
}
} // namespace openni_camera
