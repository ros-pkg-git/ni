/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
 *    Suat Gedikli <gedikli@willowgarage.com>
 *    Patrick Michelich <michelich@willowgarage.com>
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
#include <pluginlib/class_list_macros.h>
#include "openni_camera/openni_nodelet.h"
#include "openni_camera/openni_device_kinect.h"
#include "openni_camera/openni_image.h"
#include "openni_camera/openni_depth_image.h"
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <stereo_msgs/DisparityImage.h>

#if ROS_VERSION_MINIMUM(1, 3, 0)
#include <sensor_msgs/distortion_models.h>
#endif

using namespace std;
using namespace openni_wrapper;
namespace openni_camera
{
inline bool operator == (const XnMapOutputMode& mode1, const XnMapOutputMode& mode2)
{
  return (mode1.nXRes == mode2.nXRes && mode1.nYRes == mode2.nYRes && mode1.nFPS == mode2.nFPS);
}
inline bool operator != (const XnMapOutputMode& mode1, const XnMapOutputMode& mode2)
{
  return !(mode1 == mode2);
}


PLUGINLIB_DECLARE_CLASS (openni_camera, OpenNINodelet, openni_camera::OpenNINodelet, nodelet::Nodelet);

typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

OpenNINodelet::~OpenNINodelet ()
{
  device_->stopDepthStream ();
  device_->stopImageStream ();
}

void OpenNINodelet::onInit ()
{
  ros::NodeHandle comm_nh(getNodeHandle ().resolveName ("camera")); // for topics, services
  ros::NodeHandle param_nh = getPrivateNodeHandle (); // for parameters

  updateModeMaps ();      // registering mapping from config modes to XnModes and vice versa
  setupDevice (param_nh); // will change config_ to default values or user given values from param server

  param_nh.param ("rgb_frame_id", rgb_frame_id_, string (""));
  if (rgb_frame_id_.empty ())
  {
    rgb_frame_id_ = "/openni_rgb_optical_frame";
    NODELET_INFO ("'rgb_frame_id' not set. using default: '%s'", rgb_frame_id_.c_str());
  }
  else
    NODELET_INFO ("rgb_frame_id = '%s' ", rgb_frame_id_.c_str());

  param_nh.param ("depth_frame_id", depth_frame_id_, string (""));
  if (depth_frame_id_.empty ())
  {
    depth_frame_id_ = "/openni_depth_optical_frame";
    NODELET_INFO ("'depth_frame_id' not set. using default: '%s'", depth_frame_id_.c_str());
  }
  else
    NODELET_INFO ("depth_frame_id = '%s' ", depth_frame_id_.c_str());

  image_transport::ImageTransport imageTransport (comm_nh);
  image_transport::SubscriberStatusCallback subscriberChanged = boost::bind(&OpenNINodelet::subscriberChangedEvent, this);
  pub_rgb_image_   = imageTransport.advertise ("rgb/image_color", 5, subscriberChanged, subscriberChanged );
  pub_gray_image_  = imageTransport.advertise ("rgb/image_mono" , 5, subscriberChanged, subscriberChanged );
  pub_depth_image_ = imageTransport.advertise ("depth/image"    , 5, subscriberChanged, subscriberChanged );

  ros::SubscriberStatusCallback subscriberChanged2 = boost::bind(&OpenNINodelet::subscriberChangedEvent, this);
  pub_disparity_ = comm_nh.advertise<stereo_msgs::DisparityImage > ("depth/disparity", 5, subscriberChanged2, subscriberChanged2);
  pub_rgb_info_ = comm_nh.advertise<sensor_msgs::CameraInfo > ("rgb/camera_info", 5, subscriberChanged2, subscriberChanged2);
  pub_depth_info_ = comm_nh.advertise<sensor_msgs::CameraInfo > ("depth/camera_info", 5, subscriberChanged2, subscriberChanged2);
  pub_point_cloud_ = comm_nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("depth/points", 5, subscriberChanged2, subscriberChanged2);
  pub_point_cloud_rgb_ = comm_nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("rgb/points", 5, subscriberChanged2, subscriberChanged2);

  SyncPolicy sync_policy (4); // queue size
  depth_rgb_sync_.reset (new Synchronizer (sync_policy));
  depth_rgb_sync_->registerCallback (boost::bind (&OpenNINodelet::publishXYZRGBPointCloud, this, _1, _2));

  // initialize dynamic reconfigure
  reconfigure_server_.reset (new ReconfigureServer (reconfigure_mutex_, param_nh));
  reconfigure_mutex_.lock ();
  reconfigure_server_->updateConfig (config_);
  reconfigure_server_->setCallback (boost::bind (&OpenNINodelet::configCallback, this, _1, _2));
  reconfigure_mutex_.unlock ();
}

void OpenNINodelet::setupDevice (ros::NodeHandle& param_nh)
{
  // Initialize the openni device
  OpenNIDriver& driver = OpenNIDriver::getInstance ();

  if (driver.getNumberDevices () == 0)
  {
    NODELET_ERROR ("[%s] No devices connected.", getName ().c_str ());
    exit (-1);
  }

  NODELET_INFO ("[%s] Number devices connected: %d", getName ().c_str (), driver.getNumberDevices ());
  for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
  {
    NODELET_INFO ("[%s] %u. device on bus %03u:%02u is a %s (%03x) from %s (%03x) with serial id \'%s\'"
              , getName ().c_str (), deviceIdx + 1, driver.getBus (deviceIdx), driver.getAddress (deviceIdx)
              , driver.getProductName (deviceIdx), driver.getProductID (deviceIdx), driver.getVendorName (deviceIdx)
              , driver.getVendorID (deviceIdx), driver.getSerialNumber (deviceIdx));
  }

  string device_id;
  param_nh.param ("device_id", device_id, std::string ());

  try {
    if (device_id.empty ())
    {
      NODELET_WARN ("[%s] device_id is not set! Using first device.", getName ().c_str ());
      device_ = driver.getDeviceByIndex (0);
    }
    else if (device_id.find ('@') != string::npos)
    {
      size_t pos = device_id.find ('@');
      unsigned bus = atoi (device_id.substr (0, pos).c_str ());
      unsigned address = atoi (device_id.substr (pos + 1, device_id.length () - pos - 1).c_str ());
      NODELET_INFO ("[%s] searching for device with bus@address = %d@%d", getName ().c_str (), bus, address);
      device_ = driver.getDeviceByAddress (bus, address);
    }
    else if (device_id[0] == '#')
    {
      unsigned index = atoi (device_id.c_str () + 1);
      NODELET_INFO ("[%s] searching for device with index = %d", getName ().c_str (), index);
      device_ = driver.getDeviceByIndex (index - 1);
    }
    else
    {
      NODELET_INFO ("[%s] searching for device with serial number = %s", getName ().c_str (), device_id.c_str ());
      device_ = driver.getDeviceBySerialNumber (device_id);
    }
  }
  catch (const OpenNIException& exception)
  {
    if (!device_)
    {
      NODELET_ERROR ("[%s] No matching device found.", getName ().c_str ());
      exit (-1);
    }
    else
    {
      NODELET_ERROR ("[%s] could not retrieve device. Reason %s", getName ().c_str (), exception.what ());
      exit (-1);
    }
  }

  NODELET_INFO ("[%s] Opened '%s' on bus %d:%d with serial number '%s'", getName ().c_str (),
            device_->getProductName (), device_->getBus (), device_->getAddress (), device_->getSerialNumber ());

  device_->registerImageCallback (&OpenNINodelet::imageCallback, *this);
  device_->registerDepthCallback (&OpenNINodelet::depthCallback, *this);

  bool registration = false;
  param_nh.param ("depth_registration", registration, false );
  config_.depth_registration = registration;

  int debayering_method = 0;
  param_nh.param ("debayering", debayering_method, 0 );
  if(debayering_method > 2 || debayering_method < 0)
  {
    NODELET_ERROR ("Unknown debayering method %d. Only Folowing values are available: Bilinear (0), EdgeAware (1), EdgeAwareWeighted (2). Falling back to Bilinear (0).", debayering_method);
    debayering_method = 0;
  }
  config_.debayering = debayering_method;

  param_nh.param ("depth_time_offset", config_.depth_time_offset, 0.0 );
  if(config_.depth_time_offset > 1.0 || config_.depth_time_offset < -1.0)
  {
    NODELET_ERROR ("depth time offset is % 2.5f seconds. Thats unlikely... setting back to 0.0 seconds", config_.depth_time_offset);
    config_.depth_time_offset = 0.0;
  }

  param_nh.param ("image_time_offset", config_.image_time_offset, 0.0 );
  if(config_.image_time_offset > 1.0 || config_.image_time_offset < -1.0)
  {
    NODELET_ERROR ("image time offset is % 2.5f seconds. Thats unlikely... setting back to 0.0 seconds", config_.image_time_offset);
    config_.image_time_offset = 0.0;
  }

  int image_mode = mapXnMode2ConfigMode (device_->getDefaultImageMode ());
  param_nh.param ("image_mode", image_mode, image_mode );
  if (image_mode < config_.__getMin__().image_mode  || image_mode > config_.__getMax__ ().image_mode ||
      !isImageModeSupported (image_mode))
  {
    XnMapOutputMode image_md = device_->getDefaultImageMode ();
    NODELET_ERROR ("Unknown or unsopported image mode %d. Falling back to default mode %dx%d@%d.", image_mode, image_md.nXRes, image_md.nYRes, image_md.nFPS);
    image_mode = mapXnMode2ConfigMode (image_md);
  }
  config_.image_mode = image_mode;

  int depth_mode = mapXnMode2ConfigMode (device_->getDefaultDepthMode ());
  param_nh.param ("depth_mode", depth_mode, depth_mode );
  if (depth_mode < OpenNI_VGA_30Hz || depth_mode > config_.__getMax__ ().image_mode ||
      !isDepthModeSupported (depth_mode))
  {
    XnMapOutputMode depth_md = device_->getDefaultImageMode ();
    NODELET_ERROR ("Unknown or unsopported depth mode %d. Falling back to default mode %dx%d@%d.", depth_mode, depth_md.nXRes, depth_md.nYRes, depth_md.nFPS);
    depth_mode = mapXnMode2ConfigMode (depth_md);
  }
  config_.depth_mode = depth_mode;

  XnMapOutputMode image_md = mapConfigMode2XnMode ( config_.image_mode);
  image_width_  = image_md.nXRes;
  image_height_ = image_md.nYRes;

  XnMapOutputMode depth_md = mapConfigMode2XnMode ( config_.depth_mode);
  depth_width_  = depth_md.nXRes;
  depth_height_ = depth_md.nYRes;
}

void OpenNINodelet::imageCallback (const openni_wrapper::Image& image, void* cookie)
{
  ros::Time time = ros::Time::now () + ros::Duration(config_.image_time_offset);

  // mode is switching -> probably image sizes are not consistend... skip this frame
  //if (!image_mutex_.try_lock ())
  //  return;

  if (pub_rgb_info_.getNumSubscribers () > 0)
    pub_rgb_info_.publish (fillCameraInfo (time, true));

  if (pub_rgb_image_.getNumSubscribers () > 0 || pub_point_cloud_rgb_.getNumSubscribers () > 0 )
    publishRgbImage (image, time);

  if (pub_gray_image_.getNumSubscribers () > 0)
    publishGrayImage (image, time);

  //image_mutex_.unlock ();
}

void OpenNINodelet::depthCallback (const openni_wrapper::DepthImage& depth_image, void* cookie)
{
  ros::Time time = ros::Time::now () + ros::Duration(config_.depth_time_offset);
  //if (!depth_mutex_.try_lock ())
  //  return;

  // Camera info for depth image
  if (pub_depth_info_.getNumSubscribers () > 0)
    pub_depth_info_.publish (fillCameraInfo (time, false));

  // Depth image
  if (pub_depth_image_.getNumSubscribers () > 0 || (pub_point_cloud_rgb_.getNumSubscribers () > 0 ))
    publishDepthImage (depth_image, time);

  // Disparity image
  if (pub_disparity_.getNumSubscribers () > 0)
    publishDisparity (depth_image, time);

  // Unregistered point cloud
  if (pub_point_cloud_.getNumSubscribers () > 0 )
    publishXYZPointCloud (depth_image, time);

  //depth_mutex_.unlock ();
}

void OpenNINodelet::subscriberChangedEvent ()
{
  // chek if we need to start/stop any stream
  if (isImageStreamRequired () && !device_->isImageStreamRunning ())
  {
    device_->startImageStream ();
    startSynchronization ();
  }
  else if (!isImageStreamRequired () && device_->isImageStreamRunning ())
  {
    stopSynchronization ();
    device_->stopImageStream ();
    if (pub_rgb_info_.getNumSubscribers() > 0)
      NODELET_WARN("Camera Info for rgb stream has subscribers, but stream has stopped.");
  }

  if (isDepthStreamRequired () && !device_->isDepthStreamRunning ())
  {
    device_->startDepthStream ();
    startSynchronization ();
  }
  else if ( !isDepthStreamRequired () && device_->isDepthStreamRunning ())
  {
    stopSynchronization ();
    device_->stopDepthStream ();
    if (pub_depth_info_.getNumSubscribers() > 0)
      NODELET_WARN("Camera Info for depth stream has subscribers, but stream has stopped.");
  }

  // if PointcloudXYZRGB is subscribed, we have to assure that depth stream is registered and
  // image stream size is at least as big as the depth image size
  if (pub_point_cloud_rgb_.getNumSubscribers() > 0)
  {
    Config config = config_;

    reconfigure_mutex_.lock ();
    if (!device_->isDepthRegistered ())
    {
      NODELET_WARN ("turning on depth registration, since PointCloudXYZRGB has subscribers.");
      device_->setDepthRegistration (true);
      config.depth_registration = true;
    }

    XnMapOutputMode depth_mode = mapConfigMode2XnMode (config_.depth_mode);
    XnMapOutputMode image_mode = mapConfigMode2XnMode (config_.image_mode);
    if (depth_mode.nXRes > image_mode.nXRes || depth_mode.nYRes > image_mode.nYRes)
    {
      NODELET_WARN ("PointCloudXYZRGB need at least the same image size for mapping rgb values to the points");
      config.image_mode = config_.depth_mode;
    }

    reconfigure_server_->updateConfig (config);
    configCallback (config, 0);
    reconfigure_mutex_.unlock ();
  }
}

void OpenNINodelet::publishRgbImage (const Image& image, ros::Time time) const
{
  sensor_msgs::ImagePtr rgb_msg = boost::make_shared<sensor_msgs::Image > ();
  rgb_msg->header.stamp = time;
  rgb_msg->header.frame_id = rgb_frame_id_;
  rgb_msg->encoding = sensor_msgs::image_encodings::RGB8;
  rgb_msg->height = image_height_;
  rgb_msg->width = image_width_;
  rgb_msg->step = image_width_ * 3;
  rgb_msg->data.resize (rgb_msg->height * rgb_msg->step);
  image.fillRGB (rgb_msg->width, rgb_msg->height, &rgb_msg->data[0], rgb_msg->step);

  if (pub_rgb_image_.getNumSubscribers () > 0)
    pub_rgb_image_.publish (rgb_msg);
  
  if (pub_point_cloud_rgb_.getNumSubscribers () > 0)
    depth_rgb_sync_->add < 1 > (rgb_msg);
}

void OpenNINodelet::publishGrayImage (const Image& image, ros::Time time) const
{
  sensor_msgs::ImagePtr gray_msg = boost::make_shared<sensor_msgs::Image > ();
  gray_msg->header.stamp = time;
  gray_msg->header.frame_id = rgb_frame_id_;
  gray_msg->encoding = sensor_msgs::image_encodings::MONO8;
  gray_msg->height = image_height_;
  gray_msg->width = image_width_;
  gray_msg->step = image_width_;
  gray_msg->data.resize (gray_msg->height * gray_msg->step);
  image.fillGrayscale (gray_msg->width, gray_msg->height, &gray_msg->data[0], gray_msg->step);

  pub_gray_image_.publish (gray_msg);
}

void OpenNINodelet::publishDepthImage (const DepthImage& depth, ros::Time time) const
{
  sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image > ();
  depth_msg->header.stamp         = time;
  depth_msg->header.frame_id      = device_->isDepthRegistered () ? rgb_frame_id_ : depth_frame_id_;
  depth_msg->encoding             = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_msg->height               = depth_height_;
  depth_msg->width                = depth_width_;
  depth_msg->step                 = depth_msg->width * sizeof (float);
  depth_msg->data.resize (depth_msg->height * depth_msg->step);

  depth.fillDepthImage (depth_width_, depth_height_, reinterpret_cast<float*>(&depth_msg->data[0]), depth_msg->step);

  if (pub_depth_image_.getNumSubscribers () > 0)
    pub_depth_image_.publish (depth_msg);

  if (pub_point_cloud_rgb_.getNumSubscribers () > 0)
    depth_rgb_sync_->add < 0 > (depth_msg);
}

void OpenNINodelet::publishDisparity (const DepthImage& depth, ros::Time time) const
{
  stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage > ();
  disp_msg->header.stamp                  = time;
  disp_msg->header.frame_id               = device_->isDepthRegistered () ? rgb_frame_id_ : depth_frame_id_;
  disp_msg->image.header                  = disp_msg->header;
  disp_msg->image.encoding                = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_msg->image.height                  = depth_height_;
  disp_msg->image.width                   = depth_width_;
  disp_msg->image.step                    = disp_msg->image.width * sizeof (float);
  disp_msg->image.data.resize (disp_msg->image.height * disp_msg->image.step);
  disp_msg->T = depth.getBaseline ();
  disp_msg->f = depth.getFocalLength () * depth_width_ / depth.getWidth ();

  /// @todo Compute these values from DepthGenerator::GetDeviceMaxDepth() and the like
  disp_msg->min_disparity = 0.0;
  disp_msg->max_disparity = disp_msg->T * disp_msg->f / 0.3;
  disp_msg->delta_d = 0.125;

  depth.fillDisparityImage (depth_width_, depth_height_, reinterpret_cast<float*>(&disp_msg->image.data[0]), disp_msg->image.step);

  pub_disparity_.publish (disp_msg);
}

void OpenNINodelet::publishXYZPointCloud (const DepthImage& depth, ros::Time time) const
{
  const xn::DepthMetaData& depth_md = depth.getDepthMetaData ();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>() );
  cloud_msg->header.stamp = time;
  cloud_msg->height       = depth_height_;
  cloud_msg->width        = depth_width_;
  cloud_msg->is_dense     = false;

  cloud_msg->points.resize (cloud_msg->height * cloud_msg->width);

  float constant = 0.001 / device_->getDepthFocalLength (depth_width_);

  if (device_->isDepthRegistered ())
    cloud_msg->header.frame_id = rgb_frame_id_;
  else
    cloud_msg->header.frame_id = depth_frame_id_;

  float centerX = (cloud_msg->width >> 1 ) - 0.5f;
  float centerY = (cloud_msg->height >> 1) - 0.5f;

  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  unsigned depthStep = depth_md.XRes () / cloud_msg->width;
  unsigned depthSkip = (depth_md.YRes () / cloud_msg->height - 1) * depth_md.XRes ();
  int depth_idx = 0;
  pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud_msg->begin ();
  for (int v = 0; v < (int)cloud_msg->height; ++v, depth_idx += depthSkip)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u, depth_idx += depthStep, ++pt_iter)
    {
      pcl::PointXYZ& pt = *pt_iter;

      // Check for invalid measurements
      if (depth_md[depth_idx] == 0 ||
          depth_md[depth_idx] == depth.getNoSampleValue () ||
          depth_md[depth_idx] == depth.getShadowValue ())
      {
        // not valid
        pt.x = pt.y = pt.z = bad_point;
        continue;
      }

      // Fill in XYZ
      pt.x = (u - centerX) * depth_md[depth_idx] * constant;
      pt.y = (v - centerY) * depth_md[depth_idx] * constant;
      pt.z = depth_md[depth_idx] * 0.001;
    }
  }

  pub_point_cloud_.publish (cloud_msg);
}

void OpenNINodelet::publishXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg) const
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZRGB>() );
  cloud_msg->header.stamp     = depth_msg->header.stamp;
  cloud_msg->header.frame_id  = rgb_frame_id_;
  cloud_msg->height           = depth_msg->height;
  cloud_msg->width            = depth_msg->width;
  cloud_msg->is_dense         = false;

  // do not publish if rgb image is smaller than color image -> seg fault
  if (rgb_msg->height < depth_msg->height || rgb_msg->width < depth_msg->width)
  {
    // we dont want to flood the terminal with warnings
    static unsigned warned = 0;
    if (warned % 100 == 0)
      NODELET_WARN("rgb image smaller than depth image... skipping point cloud for this frame rgb:%dx%d vs. depth:%3dx%d"
              , rgb_msg->width, rgb_msg->height, depth_msg->width, depth_msg->height );
    ++warned;
    return;
  }
  cloud_msg->points.resize (cloud_msg->height * cloud_msg->width);

  float constant = 1.0f / device_->getImageFocalLength (cloud_msg->width);
  float centerX = (cloud_msg->width >> 1) - 0.5f;
  float centerY = (cloud_msg->height >> 1) - 0.5f;
  const float* depth_buffer = reinterpret_cast<const float*>(&depth_msg->data[0]);
  const uint8_t* rgb_buffer = &rgb_msg->data[0];

  // depth_msg already has the desired dimensions, but rgb_msg may be higher res.
  unsigned color_step = 3 * rgb_msg->width / cloud_msg->width;
  unsigned color_skip = 3 * (rgb_msg->height / cloud_msg->height - 1) * rgb_msg->width;
  int color_idx = 0, depth_idx = 0;
  pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud_msg->begin ();
  for (int v = 0; v < (int)cloud_msg->height; ++v, color_idx += color_skip)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u, color_idx += color_step, ++depth_idx, ++pt_iter)
    {
      pcl::PointXYZRGB& pt = *pt_iter;
      float Z = depth_buffer[depth_idx];

      // Check for invalid measurements
      if (std::isnan (Z))
      {
        pt.x = pt.y = pt.z = Z;
      }
      else
      {
        // Fill in XYZ
        pt.x = (u - centerX) * Z * constant;
        pt.y = (v - centerY) * Z * constant;
        pt.z = Z;
      }

      // Fill in color
      RGBValue color;
      color.Red   = rgb_buffer[color_idx];
      color.Green = rgb_buffer[color_idx + 1];
      color.Blue  = rgb_buffer[color_idx + 2];
      color.Alpha = 0;
      pt.rgb = color.float_value;
    }
  }

  pub_point_cloud_rgb_.publish (cloud_msg);
}

sensor_msgs::CameraInfoPtr OpenNINodelet::fillCameraInfo (ros::Time time, bool is_rgb)
{
  sensor_msgs::CameraInfoPtr info_msg = boost::make_shared<sensor_msgs::CameraInfo > ();
  info_msg->header.stamp    = time;
  info_msg->header.frame_id = is_rgb ? rgb_frame_id_ : depth_frame_id_;
  info_msg->width           = is_rgb ? image_width_ : depth_width_;
  info_msg->height          = is_rgb ? image_height_ : depth_height_;

#if ROS_VERSION_MINIMUM(1, 3, 0)
  info_msg->D = std::vector<double>(5, 0.0);
  info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
#else
  info_msg->D.assign (0.0);
#endif
  info_msg->K.assign (0.0);
  info_msg->R.assign (0.0);
  info_msg->P.assign (0.0);
  // Simple camera matrix: square pixels, principal point at center
  double f = is_rgb ? device_->getImageFocalLength (image_width_) : device_->getDepthFocalLength (depth_width_);
  info_msg->K[0] = info_msg->K[4] = f;
  info_msg->K[2] = (info_msg->width / 2) - 0.5;
  info_msg->K[5] = (info_msg->width * 3./8.) - 0.5; //aspect ratio for the camera center on kinect and presumably other devices is 4/3
  info_msg->K[8] = 1.0;
  // no rotation: identity
  info_msg->R[0] = info_msg->R[4] = info_msg->R[8] = 1.0;
  // no rotation, no translation => P=K(I|0)=(K|0)
  info_msg->P[0] = info_msg->P[5] = info_msg->K[0];
  info_msg->P[2] = info_msg->K[2];
  info_msg->P[6] = info_msg->K[5];
  info_msg->P[10] = 1.0;

  return info_msg;
}

void OpenNINodelet::configCallback (Config &config, uint32_t level)
{
  XnMapOutputMode old_image_mode = device_->getImageOutputMode ();
  XnMapOutputMode old_depth_mode = device_->getDepthOutputMode ();

  // does the device support the new image mode?
  XnMapOutputMode image_mode, compatible_image_mode;
  image_mode = mapConfigMode2XnMode (config.image_mode);

  if (!device_->findCompatibleImageMode (image_mode, compatible_image_mode))
  {
    NODELET_WARN ("Could not find any compatible image output mode for %d x %d @ %d.",
            image_mode.nXRes, image_mode.nYRes, image_mode.nFPS);

    // dont change anything!
    config = config_;
    return;
  }

  XnMapOutputMode depth_mode, compatible_depth_mode;
  depth_mode = mapConfigMode2XnMode (config.depth_mode);
  if (!device_->findCompatibleDepthMode (depth_mode, compatible_depth_mode))
  {
    NODELET_WARN ("Could not find any compatible depth output mode for %d x %d @ %d.",
            depth_mode.nXRes, depth_mode.nYRes, depth_mode.nFPS);

    // dont change anything!
    config = config_;
    return;
  }

  DeviceKinect* kinect = dynamic_cast<DeviceKinect*> (device_.get ());
  if (kinect)
  {
    switch (config.debayering)
    {
      case OpenNI_Bilinear:
        kinect->setDebayeringMethod (ImageBayerGRBG::Bilinear);
        break;
      case OpenNI_EdgeAware:
        kinect->setDebayeringMethod (ImageBayerGRBG::EdgeAware);
        break;
      case OpenNI_EdgeAwareWeighted:
        kinect->setDebayeringMethod (ImageBayerGRBG::EdgeAwareWeighted);
        break;
      default:
        NODELET_ERROR ("unknwon debayering method");
        config.debayering = config_.debayering;
        break;
    }
  }
  else if (config.debayering != config_.debayering) // this was selected explicitely
  {
    NODELET_WARN ("%s does not output bayer images. Selection has no affect.", device_->getProductName () );
  }

  if (pub_point_cloud_rgb_.getNumSubscribers () > 0)
  {
    if ( (depth_mode.nXRes > image_mode.nXRes) || (depth_mode.nYRes > image_mode.nYRes) ||
     (image_mode.nXRes % depth_mode.nXRes != 0) )
    {
      // we dont care about YRes, since SXGA works fine for kinect with all depth resolutions
      NODELET_WARN ("depth mode not compatible to image mode, since PointCloudXYZRGB has subscribers.");
      config = config_;
      return;
    }
    if (!config.depth_registration && config_.depth_registration)
    {
      NODELET_WARN ("can not turn of registration, since PointCloudXYZRGB has subscribers.");
      config = config_;
      return;
    }
  }

  // here everything is fine. Now make the changes
  if (compatible_image_mode != old_image_mode || compatible_depth_mode != old_depth_mode)
  { // streams need to be reset!
    stopSynchronization ();

    if (compatible_image_mode != old_image_mode)
    {
     // image_mutex_.lock ();
      device_->setImageOutputMode (compatible_image_mode);
      image_width_  = image_mode.nXRes;
      image_height_ = image_mode.nYRes;
      //image_mutex_.unlock ();
    }

    if (compatible_depth_mode != old_depth_mode)
    {
      //depth_mutex_.lock ();
      device_->setDepthOutputMode (compatible_depth_mode);
      depth_width_  = depth_mode.nXRes;
      depth_height_ = depth_mode.nYRes;
      //depth_mutex_.unlock ();
    }
    startSynchronization ();
  }
  else
  {
    if (config_.image_mode != config.image_mode)
    {
      image_width_  = image_mode.nXRes;
      image_height_ = image_mode.nYRes;
    }

    if (config_.depth_mode != config.depth_mode)
    {
      depth_width_  = depth_mode.nXRes;
      depth_height_ = depth_mode.nYRes;
    }
  }

  if (device_->isDepthRegistered () != config.depth_registration)
  {
    device_->setDepthRegistration (config.depth_registration);
  }

  // now we can copy
  config_ = config;
}

bool OpenNINodelet::isImageModeSupported (int image_mode) const
{
  XnMapOutputMode image_md = mapConfigMode2XnMode (image_mode);
  XnMapOutputMode compatible_mode;
  if (device_->findCompatibleImageMode (image_md, compatible_mode))
    return true;
  return false;
}

bool OpenNINodelet::isDepthModeSupported (int depth_mode) const
{
  XnMapOutputMode depth_md = mapConfigMode2XnMode (depth_mode);
  XnMapOutputMode compatible_mode;
  if (device_->findCompatibleDepthMode (depth_md, compatible_mode))
    return true;
  return false;
}

void OpenNINodelet::startSynchronization ()
{   
  if (device_->isSynchronizationSupported () && !device_->isSynchronized () &&
      device_->getImageOutputMode ().nFPS == device_->getDepthOutputMode ().nFPS &&
      device_->isImageStreamRunning () && device_->isDepthStreamRunning () )
    device_->setSynchronization (true);
}

void OpenNINodelet::stopSynchronization ()
{
  if (device_->isSynchronizationSupported () && device_->isSynchronized ())
    device_->setSynchronization (false);
}

void OpenNINodelet::updateModeMaps ()
{
  XnMapOutputMode output_mode;

  output_mode.nXRes = XN_SXGA_X_RES;
  output_mode.nYRes = XN_SXGA_Y_RES;
  output_mode.nFPS  = 15;
  xn2config_map_[output_mode] = OpenNI_SXGA_15Hz;
  config2xn_map_[OpenNI_SXGA_15Hz] = output_mode;

  output_mode.nXRes = XN_VGA_X_RES;
  output_mode.nYRes = XN_VGA_Y_RES;
  output_mode.nFPS  = 25;
  xn2config_map_[output_mode] = OpenNI_VGA_25Hz;
  config2xn_map_[OpenNI_VGA_25Hz] = output_mode;
  output_mode.nFPS  = 30;
  xn2config_map_[output_mode] = OpenNI_VGA_30Hz;
  config2xn_map_[OpenNI_VGA_30Hz] = output_mode;

  output_mode.nXRes = XN_QVGA_X_RES;
  output_mode.nYRes = XN_QVGA_Y_RES;
  output_mode.nFPS  = 25;
  xn2config_map_[output_mode] = OpenNI_QVGA_25Hz;
  config2xn_map_[OpenNI_QVGA_25Hz] = output_mode;
  output_mode.nFPS  = 30;
  xn2config_map_[output_mode] = OpenNI_QVGA_30Hz;
  config2xn_map_[OpenNI_QVGA_30Hz] = output_mode;
  output_mode.nFPS  = 60;
  xn2config_map_[output_mode] = OpenNI_QVGA_60Hz;
  config2xn_map_[OpenNI_QVGA_60Hz] = output_mode;

  output_mode.nXRes = XN_QQVGA_X_RES;
  output_mode.nYRes = XN_QQVGA_Y_RES;
  output_mode.nFPS  = 25;
  xn2config_map_[output_mode] = OpenNI_QQVGA_25Hz;
  config2xn_map_[OpenNI_QQVGA_25Hz] = output_mode;
  output_mode.nFPS  = 30;
  xn2config_map_[output_mode] = OpenNI_QQVGA_30Hz;
  config2xn_map_[OpenNI_QQVGA_30Hz] = output_mode;
  output_mode.nFPS  = 60;
  xn2config_map_[output_mode] = OpenNI_QQVGA_60Hz;
  config2xn_map_[OpenNI_QQVGA_60Hz] = output_mode;
}

int OpenNINodelet::mapXnMode2ConfigMode (const XnMapOutputMode& output_mode) const
{
  std::map<XnMapOutputMode, int, modeComp>::const_iterator it = xn2config_map_.find (output_mode);

  if (it == xn2config_map_.end ())
  {
    NODELET_ERROR ("mode %dx%d@%d could not be found", output_mode.nXRes, output_mode.nYRes, output_mode.nFPS);
    exit (-1);
  }
  else
    return it->second;
}

XnMapOutputMode OpenNINodelet::mapConfigMode2XnMode (int mode) const
{
  std::map<int, XnMapOutputMode>::const_iterator it = config2xn_map_.find (mode);
  if (it == config2xn_map_.end ())
  {
    NODELET_ERROR ("mode %d could not be found", mode);
    exit (-1);
  }
  else
    return it->second;
}
}
