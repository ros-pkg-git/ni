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
#include <vector>

using namespace std;

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

/** \brief Constructor */
OpenNIDriver::OpenNIDriver (ros::NodeHandle comm_nh, ros::NodeHandle param_nh)
  : comm_nh_ (comm_nh)
  , param_nh_ (param_nh)
  , reconfigure_server_ (param_nh)
  , started_ (false)
  , shadow_value_ (0)
  , no_sample_value_ (0)
{
  cout << "OpenNIDriver::OpenNIDriver" << endl;
  // Init the OpenNI context
  XnStatus status = context_.Init ();

  if (status != XN_STATUS_OK)
  {
    ROS_ERROR ("[OpenNIDriver] Init: %s", xnGetStatusString (status));
    return;
  }
  ROS_INFO ("[OpenNIDriver] Initialization successful.");

  // Set up reconfigure server
  ReconfigureServer::CallbackType f = boost::bind(&OpenNIDriver::configCb, this, _1, _2);
  reconfigure_server_.setCallback(f);
  
  // Assemble the point cloud data
  std::string openni_depth_frame;
  //param_nh.param ("openni_depth_frame", openni_depth_frame, std::string ("/openni_depth_frame")); //Publish into optical frame as it has z forward.  
  param_nh.param ("openni_depth_optical_frame", openni_depth_frame, std::string ("/openni_depth_optical_frame"));
  // @Radu: is there any point in still publishing sensor_msgs/PointCloud? Don't we want to deprecate this at some point?

  /// @todo "u" and "v" channels?
  cloud2_.header.frame_id = openni_depth_frame;
  

  // set frame ids
  depth_image_.header.frame_id = openni_depth_frame;

  std::string openni_RGB_frame;
  //  param_nh.param ("openni_rgb_frame", openni_RGB_frame, std::string ("/openni_rgb_frame"));
  param_nh.param ("openni_rgb_optical_frame", openni_RGB_frame, std::string ("/openni_rgb_optical_frame"));
  rgb_image_.header.frame_id = openni_RGB_frame;

  // IR is the same as the depth frame.  
  ir_image_.header.frame_id = openni_depth_frame;

  // Publishers and subscribers
  image_transport::ImageTransport it(comm_nh);
  pub_rgb_     = it.advertiseCamera ("rgb/image_raw", 15);
  pub_depth_   = it.advertiseCamera ("depth/image_raw", 15);
  pub_ir_      = it.advertiseCamera ("ir/image_raw", 15);
  pub_depth_points2_ = comm_nh.advertise<sensor_msgs::PointCloud2>("depth/points2", 15);

  cout << "OpenNIDriver::OpenNIDriver...end" << endl;
}

/** \brief Destructor */
OpenNIDriver::~OpenNIDriver ()
{
  stop ();
  context_.Shutdown ();
}

/** \brief Initialize an OpenNI device, given an index.
  * \param index the index of the device to initialize
  */
bool
OpenNIDriver::init (int index)
{
  // TODO: switch between several devices based on the index
  // (the current OpenNI interface doesn't support this atm)

  // Create a DepthGenerator node
  //updateDeviceSettings ();

  return (true);
}

/** \brief Start (resume) the data acquisition process. */
bool
OpenNIDriver::start ()
{
  cout << "OpenNIDriver::start" << endl;
  started_ = false;
  // Make OpenNI start generating data

  std::cout << "start depth stream..." << std::flush;
  XnStatus status = depth_generator_.StartGenerating();
  if (status != XN_STATUS_OK)
  {
    ROS_ERROR ("[OpenNIDriver::start] Error in start (): %s", xnGetStatusString (status));
    return (false);
  }

  
  if (config_.image_type == 0)
  {
    std::cout << "OK\nstart image stream..." << std::flush;
    status = image_generator_.StartGenerating();
    if (status != XN_STATUS_OK)
    {
      ROS_ERROR ("[OpenNIDriver::start] Error in start (): %s", xnGetStatusString (status));
      return (false);
    }

    std::cout << "OK\nturning registration on..." << std::flush;
    // Note: For the PSDK5 device, the alternative viewpoint needs to be set after starting
    // data generation, or the depth generation hangs.
    depth_generator_.GetAlternativeViewPointCap().SetViewPoint( image_generator_ );
    std::cout << "OK" << std::endl;
  }
  else if(config_.image_type == 1)
  {
    std::cout << "OK\nstart infrared stream..." << std::flush;
    status = ir_generator_.StartGenerating();
    if (status != XN_STATUS_OK)
    {
      ROS_ERROR ("[OpenNIDriver::start] Error in start (): %s", xnGetStatusString (status));
      return (false);
    }
    std::cout << "OK" << std::endl;
  }

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

/** \brief Spin loop. */
bool 
OpenNIDriver::spin ()
{
  cout << "OpenNIDriver::spin" << endl;
  ros::Duration r (0.01);

  while (comm_nh_.ok ())
  {  
    if (pub_rgb_.getNumSubscribers () == 0 && pub_depth_.getNumSubscribers () == 0 && pub_depth_points2_.getNumSubscribers () == 0)
    {
      if (started_)
      {
        context_.StopGeneratingAll ();
        started_ = false;
      }
      r.sleep ();
      continue;
    }

    if (!started_)
    {
      start(); //context_.StartGeneratingAll ();
      started_ = true;
    }

    // Wait for new data to be available
    XnStatus status = context_.WaitAndUpdateAll ( );
    //rc_ = context_.WaitAnyUpdateAll ();
    if (status != XN_STATUS_OK)
    {
      ROS_ERROR ("[OpenNIDriver::spin] Error receiving data: %s", xnGetStatusString (status));
      continue;
    }

    publish ();

    // Spin for ROS message processing
    ros::spinOnce ();
  }
  return (true);
}

/** \brief Take care of putting data in the correct ROS message formats. */
void 
OpenNIDriver::publish ()
{
  static bool first_publish = true;
  static ros::Duration time_offset;
  if (first_publish)
  {
    ros::Time ros_time = ros::Time::now ();
    vector<XnUInt64> time_stamps;
    time_stamps.push_back( depth_generator_.GetTimestamp () );

    if (image_generator_.IsValid())
      time_stamps.push_back( image_generator_.GetTimestamp () );

    if (ir_generator_.IsValid())
      time_stamps.push_back( ir_generator_.GetTimestamp () );
    
    XnUInt64 latest_time = *(std::max_element( time_stamps.begin(), time_stamps.end() ));

    ros::Time latest (latest_time / 1000000, (latest_time % 1000000) * 1000);

    time_offset = ros_time - latest;

    first_publish = false;
    //ROS_INFO ( "ros time: %f, latest time: %f, time diff %f", ros_time.toSec(), latest.toSec(), time_offset.toSec() );
  }

  xn::DepthMetaData depth_md_;
	depth_generator_.GetMetaData (depth_md_);

  if (config_.image_type == 0)
  {
    const XnUInt8* rgb_buffer   = image_generator_.GetImageMap ();

    // Fill raw RGB image message
    if (pub_rgb_.getNumSubscribers () > 0)
    {
      ros::Time time (image_generator_.GetTimestamp () / 1000000, (image_generator_.GetTimestamp () % 1000000) * 1000);
      time += time_offset;
      //ROS_INFO ( "color image %i (%i) -> %f", image_generator_.GetFrameID(), image_generator_.GetTimestamp(), time.toSec() );
      rgb_image_.header.stamp = rgb_info_.header.stamp = time;
      rgb_image_.header.seq = image_generator_.GetFrameID ();
      memcpy (&rgb_image_.data[0], &rgb_buffer[0], rgb_image_.data.size ());
      pub_rgb_.publish (boost::make_shared<const sensor_msgs::Image> (rgb_image_),
                        boost::make_shared<const sensor_msgs::CameraInfo> (rgb_info_));
    }
  }
  else if (config_.image_type == 1)
  {
    std::cout << "filling IR image..." << std::flush;
    ros::Time time (ir_generator_.GetTimestamp () / 1000000, (ir_generator_.GetTimestamp () % 1000000) * 1000);
    time += time_offset;
    const XnIRPixel* ir_map =	ir_generator_.GetIRMap ();
    ir_image_.header.stamp = ir_info_.header.stamp = time;
    ir_image_.header.seq = ir_generator_.GetFrameID ();
    memcpy (&ir_image_.data[0], ir_map, ir_image_.data.size ());

    std::cout << "OK\npublishing IR image..." << std::flush;
    pub_ir_.publish(boost::make_shared<const sensor_msgs::Image> (ir_image_),
                    boost::make_shared<const sensor_msgs::CameraInfo> (ir_info_));
    std::cout<<"OK\n";
  }

  // Fill in the PointCloud2 structure
  if (pub_depth_points2_.getNumSubscribers () > 0)
  {
    XnUInt64 latest_time = std::max (depth_generator_.GetTimestamp (), image_generator_.GetTimestamp ());
    ros::Time time (latest_time / 1000000, (latest_time % 1000000) * 1000);
    time += time_offset;
    
    cloud2_.header.stamp = time;
    cloud2_.header.seq = std::max (image_generator_.GetFrameID (), depth_generator_.GetFrameID ());

    // Assemble an awesome sensor_msgs/PointCloud2 message
    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    int k = 0;
    
    float* pt_data = reinterpret_cast<float*>(&cloud2_.data[0]);
    float constant = pixel_size_ * 0.001 / F_;
    

    unsigned xStep = depth_md_.XRes () / cloud2_.width;
    unsigned ySkip = (depth_md_.YRes () / cloud2_.height - 1) * depth_md_.XRes ();

    int centerX = cloud2_.width >> 1;
    int centerY = cloud2_.height >> 1;
    constant *= xStep;
    
    for (register int v = 0; v < cloud2_.height; ++v, k += ySkip)
    {
      for (register int u = 0; u < cloud2_.width; ++u, k += xStep, pt_data += cloud2_.fields.size())
      {
        // Check for invalid measurements
        if (depth_md_[k] == 0 || depth_md_[k] == no_sample_value_ || depth_md_[k] == shadow_value_)
        {
          // not valid
          pt_data[0] = bad_point;
          pt_data[1] = bad_point;
          pt_data[2] = bad_point;
          continue;
        }

        // Fill in XYZ
        pt_data[0] = (u - centerX) * depth_md_[k] * constant;
        pt_data[1] = (v - centerY) * depth_md_[k] * constant;
        pt_data[2] = depth_md_[k] * 0.001;

        // Fill in color
        //color.Red   = rgb_buffer[k * 3 ];
        //color.Green = rgb_buffer[k * 3 + 1];
        //color.Blue  = rgb_buffer[k * 3 + 2];
        //pt_data[3] = color.float_value;
      }
    }

    // fill in color information

    if (config_.point_cloud_type == 1)
    {
      // rewind!
      k = 0;
      pt_data = reinterpret_cast<float*>(&cloud2_.data[0]);
      const XnUInt8* rgb_buffer = image_generator_.GetImageMap ();
      RGBValue color;
      color.Alpha = 0;
      
      int XStep = 0; ySkip = 0;
      if (config_.image_resolution == 0)
      {
        xStep = 640 / cloud2_.width;
        ySkip = (480 / cloud2_.height - 1) * 640;
      }
      else if (config_.image_resolution == 1)
      {
        xStep = 1280 / cloud2_.width;
        ySkip = (1024 / cloud2_.height - 1) * 1280;
      }

      xStep *= 3;
      ySkip *= 3;
      for (register int v = 0; v < cloud2_.height; ++v, k += ySkip)
      {
        for (register int u = 0; u < cloud2_.width; ++u, k += xStep, pt_data += cloud2_.fields.size())
        {
          color.Red   = rgb_buffer[k];
          color.Green = rgb_buffer[k+1];
          color.Blue  = rgb_buffer[k+2];
          pt_data[3] = color.float_value;
        }
      }
    }
    
    pub_depth_points2_.publish (boost::make_shared<const sensor_msgs::PointCloud2> (cloud2_));
  }
 
  if (pub_depth_.getNumSubscribers () > 0)
  {
    ros::Time time (image_generator_.GetTimestamp () / 1000000, (image_generator_.GetTimestamp () % 1000000) * 1000);
    time += time_offset;
    //ROS_INFO ( "depth image %i (%i) -> %f", depth_generator_.GetFrameID(), depth_generator_.GetTimestamp(), time.toSec() );

    depth_image_.header.stamp = depth_info_.header.stamp = time;
    depth_image_.header.seq = depth_generator_.GetFrameID ();
    // Fill in the depth image data
    // iterate over all elements and fill disparity matrix: disp[x,y] = f * b / z_distance[x,y];
    float constant = focal_length_ * baseline_ * 1000.0;
    float* pixel = reinterpret_cast<float*>(&depth_image_.data[0]);
    for (register int i = 0; i < 640 * 480; ++i, ++pixel)
    {
      if (depth_md_[i] == 0 || depth_md_[i] == no_sample_value_ || depth_md_[i] == shadow_value_)
        *pixel = 0.0;
      else
        *pixel = constant / (double)depth_md_[i];
    }
    
    /*float sum_depth = 0;
		for (register int i = 0; i < 640; ++i, ++pixel)
    {
      sum_depth += depth_md_[i];
    }*/
		
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


void OpenNIDriver::configCb (Config &config, uint32_t level)
{
  cout << "OpenNIDriver::configCb" << endl;
  config.image_type = 0;
  //config.image_resolution = 0;

  config_ = config;
  stop();
  updateDeviceSettings();
  start();
}

bool OpenNIDriver::updateDeviceSettings()
{
  unsigned width, height, fps;
  switch( config_.image_resolution )
  {
    case 0: width   = 640;
            height  = 480;
            fps     = 30;
      break;
    case 1: width   = 1280;
            height  = 1024;
            fps     = 15;
      break;
    default:ROS_WARN("Unknwon image resolution - falling back to VGA@30Hz");
            width   = 640;
            height  = 480;
            fps     = 30;
      break;
  }

  ImageType image_type;
  switch( config_.image_type )
  {
    case 0: image_type = RGB888; break;
    case 1: image_type = IR; break;

    default: ROS_WARN("Unknwon image format - falling back to RGB888");
             image_type = RGB888;
      break;
  }

  cout << "pc res: " << config_.point_cloud_resolution << endl;
  switch( config_.point_cloud_resolution )
  {
    case 0: cloud2_.height = 120;
            cloud2_.width  = 160;
            break;

    case 1: cloud2_.height = 240;
            cloud2_.width  = 320;
            break;

    case 2: cloud2_.height = 480;
            cloud2_.width  = 640;
            break;
    default: ROS_WARN("Unknwon point cloud size format - falling back to VGA");
            cloud2_.height = 480;
            cloud2_.width  = 640;
            break;
  }

  cout << "pc type: " << config_.point_cloud_type << endl;

  if (config_.point_cloud_type==1)
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
  cloud2_.row_step   = cloud2_.point_step * cloud2_.width;
  cloud2_.data.resize (cloud2_.row_step   * cloud2_.height);
  cloud2_.is_dense = false;

  // Assemble the depth image data
  depth_image_.height = 480;
  depth_image_.width = 640;
  depth_image_.encoding = "32FC1";
  depth_image_.step = 640 * sizeof (float);
  depth_image_.data.resize (depth_image_.step * depth_image_.height);

  // Assemble the image data
  rgb_image_.height = height;
  rgb_image_.width = width;
  rgb_info_.header.frame_id = rgb_image_.header.frame_id; 
  rgb_image_.encoding = sensor_msgs::image_encodings::RGB8;
  rgb_image_.data.resize (width * height * 3);
  rgb_image_.step = width * 3;

  ir_info_.header.frame_id = ir_image_.header.frame_id;
  ir_image_.height = height;
  ir_image_.width = width;
  ir_image_.encoding = sensor_msgs::image_encodings::MONO16;
  ir_image_.data.resize (width * height << 1);
  ir_image_.step = (width << 1);

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
    else
      ROS_INFO_STREAM ("[OpenNIDriver] Pixel size: " << pixel_size_);

    pixel_size_ *= 2.0;

    if (depth_generator_.GetIntProperty ("ZPD", F_) != XN_STATUS_OK)
      ROS_ERROR ("[OpenNIDriver] Could not read virtual plane distance!");
    else
      ROS_INFO_STREAM ("[OpenNIDriver] Virtual plane distance: " << F_);

    if (depth_generator_.GetRealProperty ("LDDIS", baseline_) != XN_STATUS_OK)
      ROS_ERROR ("[OpenNIDriver] Could not read base line!");
    else
      ROS_INFO_STREAM ("[OpenNIDriver] Base line: " << baseline_);

    // baseline from cm -> meters
    baseline_ *= 0.01;

    //focal length from mm -> pixels
    focal_length_ = (double)F_/pixel_size_;

    if (depth_generator_.GetIntProperty ("ShadowValue", shadow_value_) != XN_STATUS_OK)
      ROS_WARN ("[OpenNIDriver] Could not read shadow value!");

    if (depth_generator_.GetIntProperty ("NoSampleValue", no_sample_value_) != XN_STATUS_OK)
      ROS_WARN ("[OpenNIDriver] Could not read no sample value!");
  }

  if (config_.image_type == RGB888)
  {
    while (ir_generator_.IsValid())
    {
      std::cout << "destorying ir generator" << std::endl;
      ir_generator_.Unref();
    }
    if (!image_generator_.IsValid())
    {
      rc_ = image_generator_.Create (context_);

      if (rc_ != XN_STATUS_OK)
      {
        ROS_ERROR ("[OpenNIDriver] Failed to create ImageGenerator: %s", xnGetStatusString (rc_));
        return (false);
      }
    }
    mode.nXRes = width;
    mode.nYRes = height;
    mode.nFPS  = fps;
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
  }
  else //IR
  {
    while (image_generator_.IsValid())
    {
      std::cout << "destorying image generator" << std::endl;
      image_generator_.Unref();
    }
    
    if (!ir_generator_.IsValid())
    {
      rc_ = ir_generator_.Create (context_);

      if (rc_ != XN_STATUS_OK)
      {
        ROS_ERROR ("[OpenNIDriver] Failed to create IRGenerator: %s", xnGetStatusString (rc_));
        return (false);
      }
    }
    mode.nXRes = width;
    mode.nYRes = height;
    mode.nFPS  = fps;
    if (ir_generator_.SetMapOutputMode (mode) != XN_STATUS_OK)
    {
      ROS_ERROR("[OpenNIDriver] Failed to set ir output mode");
      return (false);
    }
  }
  
  /// @todo Just have a "kinect_mode" parameter to flip all the switches?
  // RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS
  int registration_type = 0;
  if (param_nh_.getParam ("registration_type", registration_type))
  {
    if (depth_generator_.SetIntProperty ("RegistrationType", registration_type) != XN_STATUS_OK)
      ROS_WARN ("[OpenNIDriver] Error enabling registration!");
  }
}

} // namespace openni_camera
