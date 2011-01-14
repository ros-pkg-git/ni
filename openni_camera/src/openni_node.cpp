/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010
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

/// @todo Organize includes
#include <openni_camera/openni_driver.h>
#include <openni_camera/openni_device.h>
#include <sstream>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <boost/make_shared.hpp>
#include <XnContext.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/Imu.h>
#include <stereo_msgs/DisparityImage.h>

#include <ros/ros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <openni_camera/OpenNIConfig.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Branch on whether we have the changes to CameraInfo in unstable
#if ROS_VERSION_MINIMUM(1, 3, 0)
#include <sensor_msgs/distortion_models.h>
#endif

using namespace openni_wrapper;
using namespace std;
using namespace ros;

namespace openni_camera
{

class OpenNINode
{
public:
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

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
public:
  OpenNINode (NodeHandle comm_nh, NodeHandle param_nh, boost::shared_ptr<OpenNIDevice> device, const string& topic);
  ~OpenNINode ();
  void run ();

protected:
  typedef OpenNIConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  
  void imageCallback (const Image& image, void* cookie);
  void depthCallback (const DepthImage& depth, void* cookie);

  sensor_msgs::CameraInfoPtr fillCameraInfo (ros::Time time, bool is_rgb);
  void publishRgbImageColor (const Image& image, ros::Time time);
  void publishRgbImageMono (const Image& image, ros::Time time);
  void publishDepthInfo (const DepthImage& depth, ros::Time time);
  void publishDepthImage (const DepthImage& depth, ros::Time time);
  void publishDisparity (const DepthImage& depth, ros::Time time);
  void publishXYZPointCloud (const DepthImage& depth, ros::Time time);
  void publishXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg,
                                const sensor_msgs::ImageConstPtr& rgb_msg);
  
  void configCallback (Config &config, uint32_t level);
  void updateDeviceSettings (const Config &config); /// @todo Not implemented
  int mapMode (const XnMapOutputMode& output_mode) const;
  void mapMode (int mode, XnMapOutputMode& output_mode) const;
  unsigned getFPS (int mode) const;
  unsigned image_width_;
  unsigned image_height_;
  unsigned depth_width_;
  unsigned depth_height_;
  boost::shared_ptr<OpenNIDevice> device_;
  ros::Publisher pub_rgb_info_, pub_depth_info_;
  image_transport::Publisher pub_rgb_image_, pub_gray_image_, pub_depth_image_;
  ros::Publisher pub_disparity_;
  ros::Publisher pub_point_cloud_;

  // Approximate synchronization for XYZRGB point clouds.
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  boost::shared_ptr<Synchronizer> depth_rgb_sync_;

  string topic_;
  NodeHandle comm_nh_;
  NodeHandle param_nh_;

  // Dynamic reconfigure
  ReconfigureServer reconfigure_server_;
  Config config_;

  static const string rgb_frame_id_;
  static const string depth_frame_id_;
};

/// @todo These need to be parameters
const string OpenNINode::rgb_frame_id_ = "openni_rgb_optical_frame";
const string OpenNINode::depth_frame_id_ = "openni_depth_optical_frame";

OpenNINode::OpenNINode (NodeHandle comm_nh, NodeHandle param_nh,
                        boost::shared_ptr<OpenNIDevice> device, const string& topic)
  : device_ (device)
  , topic_ (topic)
  , comm_nh_ (comm_nh)
  , param_nh_ (param_nh)
  , reconfigure_server_ (param_nh)
{
  ReconfigureServer::CallbackType reconfigure_callback =
    boost::bind (&OpenNINode::configCallback, this, _1, _2);
  reconfigure_server_.setCallback (reconfigure_callback);

  image_transport::ImageTransport image_transport (comm_nh);
  /// @todo 15 looks like overkill for the queue size
  pub_rgb_info_    = comm_nh_.advertise<sensor_msgs::CameraInfo> ("rgb/camera_info", 15);
  pub_depth_info_  = comm_nh_.advertise<sensor_msgs::CameraInfo> ("depth/camera_info", 15);
  pub_rgb_image_   = image_transport.advertise ("rgb/image_color", 15);
  pub_gray_image_  = image_transport.advertise ("rgb/image_mono", 15);
  pub_depth_image_ = image_transport.advertise ("depth/image", 15);
  pub_disparity_   = comm_nh_.advertise<stereo_msgs::DisparityImage > ("depth/disparity", 15);
  pub_point_cloud_ = comm_nh.advertise<PointCloud > ("depth/points2", 15);

  /// @todo Set inter-message lower bound, age penalty, max interval to lower latency
  SyncPolicy sync_policy (4); // queue size
  // Connect no inputs, we'll add messages manually
  depth_rgb_sync_.reset (new Synchronizer (sync_policy));
  depth_rgb_sync_->registerCallback (boost::bind (&OpenNINode::publishXYZRGBPointCloud,
                                                  this, _1, _2));

  /// @todo Is this done in configCallback?
  XnMapOutputMode output_mode;
  device_->getImageOutputMode (output_mode);
  image_width_ = output_mode.nXRes;
  image_height_ = output_mode.nYRes;

  device_->getDepthOutputMode (output_mode);
  depth_width_ = output_mode.nXRes;
  image_height_ = output_mode.nYRes;

  // registering callback functions
  device_->registerImageCallback (&OpenNINode::imageCallback, *this, NULL);
  device_->registerDepthCallback (&OpenNINode::depthCallback, *this, NULL);

  /// @todo Start and stop as needed
  device_->startImageStream ();
  device_->startDepthStream ();
}

OpenNINode::~OpenNINode ()
{
  /// @todo Need to totally stop device here, or can have callbacks invoked after
  /// we've already started destroying stuff
}

void OpenNINode::imageCallback (const Image& image, void* cookie)
{
  /// @todo Separate this into smaller functions
  /// @todo Some sort of offset based on the device timestamp
  ros::Time time = ros::Time::now();

  if (pub_rgb_info_.getNumSubscribers () > 0)
    pub_rgb_info_.publish (fillCameraInfo (time, true));

  if (pub_rgb_image_.getNumSubscribers () > 0 ||
      (pub_point_cloud_.getNumSubscribers () > 0 &&
       config_.point_cloud_type == OpenNI_XYZRGB))
    publishRgbImageColor (image, time);
  
  if (pub_gray_image_.getNumSubscribers () > 0)
    publishRgbImageMono (image, time);
}

void OpenNINode::depthCallback (const DepthImage& depth, void* cookie)
{
  /// @todo Some sort of offset based on the device timestamp
  ros::Time time = ros::Time::now();

  // Camera info for depth image
  if (pub_depth_info_.getNumSubscribers () > 0)
    pub_depth_info_.publish (fillCameraInfo (time, device_->isDepthRegistered ()));
  
  // Depth image
  if (pub_depth_image_.getNumSubscribers () > 0 ||
      (pub_point_cloud_.getNumSubscribers () > 0 &&
       config_.point_cloud_type == OpenNI_XYZRGB))
    publishDepthImage (depth, time);

  // Disparity image
  if (pub_disparity_.getNumSubscribers () > 0)
    publishDisparity (depth, time);

  // Unregistered point cloud
  if (pub_point_cloud_.getNumSubscribers () > 0 &&
      config_.point_cloud_type != OpenNI_XYZRGB)
    publishXYZPointCloud(depth, time);
}

sensor_msgs::CameraInfoPtr OpenNINode::fillCameraInfo (ros::Time time, bool is_rgb)
{
  sensor_msgs::CameraInfoPtr info_msg = boost::make_shared<sensor_msgs::CameraInfo>();
  info_msg->header.stamp    = time;
  info_msg->header.frame_id = is_rgb ? rgb_frame_id_ : depth_frame_id_;
  info_msg->width  = is_rgb ? image_width_  : depth_width_;
  info_msg->height = is_rgb ? image_height_ : depth_height_;
  // No distortion (yet!)
#if ROS_VERSION_MINIMUM(1, 3, 0)
  info_msg->D = std::vector<double>( 5, 0.0 );
  info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
#else
  info_msg->D.assign( 0.0 );
#endif
  info_msg->K.assign( 0.0 );
  info_msg->R.assign( 0.0 );
  info_msg->P.assign( 0.0 );
  // Simple camera matrix: square pixels, principal point at center
  double f = is_rgb ? device_->getImageFocalLength(image_width_) :
                      device_->getDepthFocalLength(depth_width_);
  info_msg->K[0] = info_msg->K[4] = f;
  info_msg->K[2] = (info_msg->width  / 2) - 0.5;
  info_msg->K[5] = (info_msg->height / 2) - 0.5;
  info_msg->K[8] = 1.0;
  // no rotation: identity
  info_msg->R[0] = info_msg->R[4] = info_msg->R[8] = 1.0;
  // no rotation, no translation => P=K(I|0)=(K|0)
  info_msg->P[0]    = info_msg->P[5] = info_msg->K[0];
  info_msg->P[2]    = info_msg->K[2];
  info_msg->P[6]    = info_msg->K[5];
  info_msg->P[10]   = 1.0;

  return info_msg;
}

void OpenNINode::publishRgbImageColor (const Image& image, ros::Time time)
{
  sensor_msgs::ImagePtr rgb_msg = boost::make_shared<sensor_msgs::Image> ();
  rgb_msg->header.stamp    = time;
  rgb_msg->header.frame_id = rgb_frame_id_;
  rgb_msg->encoding = sensor_msgs::image_encodings::RGB8;
  rgb_msg->height   = image_height_;
  rgb_msg->width    = image_width_;
  rgb_msg->step     = image_width_ * 3;
  rgb_msg->data.resize (rgb_msg->height * rgb_msg->step);
  image.fillRGB(rgb_msg->width, rgb_msg->height, &rgb_msg->data[0], rgb_msg->step);

  pub_rgb_image_.publish(rgb_msg);
  if (pub_point_cloud_.getNumSubscribers () > 0 &&
      config_.point_cloud_type == OpenNI_XYZRGB)
    depth_rgb_sync_->add<1>(rgb_msg);
}

void OpenNINode::publishRgbImageMono (const Image& image, ros::Time time)
{
  sensor_msgs::ImagePtr gray_msg = boost::make_shared<sensor_msgs::Image> ();
  gray_msg->header.stamp    = time;
  gray_msg->header.frame_id = rgb_frame_id_;
  gray_msg->encoding = sensor_msgs::image_encodings::MONO8;
  gray_msg->height   = image_height_;
  gray_msg->width    = image_width_;
  gray_msg->step     = image_width_;
  gray_msg->data.resize (gray_msg->height * gray_msg->step);
  image.fillGrayscale(gray_msg->width, gray_msg->height, &gray_msg->data[0],
                      gray_msg->step);

  pub_gray_image_.publish(gray_msg);
}

void OpenNINode::publishDepthImage (const DepthImage& depth, ros::Time time)
{
  sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image> ();
  depth_msg->header.stamp    = time;
  depth_msg->header.frame_id = device_->isDepthRegistered () ? rgb_frame_id_ :
                                                               depth_frame_id_;
  depth_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_msg->height   = depth_height_;
  depth_msg->width    = depth_width_;
  depth_msg->step     = depth_msg->width * sizeof(float);
  depth_msg->data.resize (depth_msg->height * depth_msg->step);

  depth.fillDepthImage(depth_msg->width, depth_msg->height,
                       reinterpret_cast<float*>(&depth_msg->data[0]),
                       depth_msg->step);

  pub_depth_image_.publish (depth_msg);
  
  if (pub_point_cloud_.getNumSubscribers () > 0 &&
      config_.point_cloud_type == OpenNI_XYZRGB)
    depth_rgb_sync_->add<0>(depth_msg);
}

void OpenNINode::publishDisparity (const DepthImage& depth, ros::Time time)
{
  stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage> ();
  disp_msg->header.stamp    = time;
  disp_msg->header.frame_id = device_->isDepthRegistered () ? rgb_frame_id_ :
                                                              depth_frame_id_;
  disp_msg->image.header = disp_msg->header;
  disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_msg->image.height   = depth_height_;
  disp_msg->image.width    = depth_width_;
  disp_msg->image.step     = disp_msg->image.width * sizeof(float);
  disp_msg->image.data.resize (disp_msg->image.height * disp_msg->image.step);
  disp_msg->T = depth.getBaseline ();
  disp_msg->f = depth.getFocalLength () * depth_width_ / depth.getWidth ();
  /// @todo Compute these values from DepthGenerator::GetDeviceMaxDepth() and the like
  disp_msg->min_disparity = 0.0;
  disp_msg->max_disparity = disp_msg->T * disp_msg->f / 0.3;
  disp_msg->delta_d = 0.125;

  depth.fillDisparityImage(disp_msg->image.width, disp_msg->image.height,
                           reinterpret_cast<float*>(&disp_msg->image.data[0]),
                           disp_msg->image.step);

  pub_disparity_.publish (disp_msg);
}

void OpenNINode::publishXYZPointCloud (const DepthImage& depth, ros::Time time)
{
  const xn::DepthMetaData& depth_md = depth.getDepthMetaData ();
  
  PointCloud::Ptr cloud_msg = boost::make_shared<PointCloud> ();
  cloud_msg->header.stamp = time;
  cloud_msg->height = depth_height_;
  cloud_msg->width  = depth_width_;
  cloud_msg->is_dense = false;

  cloud_msg->points.resize(cloud_msg->height * cloud_msg->width);
  
  float constant;
  if (device_->isDepthRegistered ())
  {
    constant = 0.001 / device_->getImageFocalLength(depth_width_);
    cloud_msg->header.frame_id = rgb_frame_id_;
  }
  else
  {
    constant = 0.001 / device_->getDepthFocalLength(depth_width_);
    cloud_msg->header.frame_id = depth_frame_id_;
  }

  float centerX = (cloud_msg->width  / 2) - 0.5f;
  float centerY = (cloud_msg->height / 2) - 0.5f;

  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  
  unsigned depthStep = depth_md.XRes () / cloud_msg->width;
  unsigned depthSkip = (depth_md.YRes () / cloud_msg->height - 1) * depth_md.XRes ();
  int depth_idx = 0;
  PointCloud::iterator pt_iter = cloud_msg->begin();
  for (int v = 0; v < (int)cloud_msg->height; ++v, depth_idx += depthSkip)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u, depth_idx += depthStep, ++pt_iter)
    {
      pcl::PointXYZRGB& pt = *pt_iter;
      
      // Check for invalid measurements
      if (depth_md[depth_idx] == 0 ||
          depth_md[depth_idx] == depth.getNoSampleValue () ||
          depth_md[depth_idx] == depth.getShadowValue ())
      {
        // not valid
        pt.x = pt.y = pt.z = pt.rgb = bad_point;
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

void OpenNINode::publishXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg,
                                          const sensor_msgs::ImageConstPtr& rgb_msg)
{
  PointCloud::Ptr cloud_msg = boost::make_shared<PointCloud> ();
  cloud_msg->header.stamp    = depth_msg->header.stamp;
  cloud_msg->header.frame_id = rgb_frame_id_;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width  = depth_msg->width;
  cloud_msg->is_dense = false;

  cloud_msg->points.resize(cloud_msg->height * cloud_msg->width);
  
  float constant = 1.0f / device_->getImageFocalLength(cloud_msg->width);
  float centerX = (cloud_msg->width  / 2) - 0.5f;
  float centerY = (cloud_msg->height / 2) - 0.5f;
  const float* depth_buffer = reinterpret_cast<const float*>(&depth_msg->data[0]);
  const uint8_t* rgb_buffer = &rgb_msg->data[0];

  // depth_msg already has the desired dimensions, but rgb_msg may be higher res.
  unsigned color_step = 3 * rgb_msg->width / cloud_msg->width;
  unsigned color_skip = 3 * (rgb_msg->height / cloud_msg->height - 1) * rgb_msg->width;
  int color_idx = 0, depth_idx = 0;
  PointCloud::iterator pt_iter = cloud_msg->begin();
  for (int v = 0; v < (int)cloud_msg->height; ++v, color_idx += color_skip)
  {
    for (int u = 0; u < (int)cloud_msg->width;
         ++u, color_idx += color_step, ++depth_idx, ++pt_iter)
    {
      pcl::PointXYZRGB& pt = *pt_iter;
      float Z = depth_buffer[depth_idx];

      // Check for invalid measurements
      if (std::isnan(Z))
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

  pub_point_cloud_.publish(cloud_msg);
}

int OpenNINode::mapMode (const XnMapOutputMode& output_mode) const
{
  if (output_mode.nXRes == 1280 && output_mode.nYRes == 1024)
    return OpenNI_SXGA_15Hz;
  else if (output_mode.nXRes == 640 && output_mode.nYRes == 480)
    return OpenNI_VGA_30Hz;
  else if (output_mode.nXRes == 320 && output_mode.nYRes == 240)
    return OpenNI_QVGA_30Hz;
  else if (output_mode.nXRes == 160 && output_mode.nYRes == 120)
    return OpenNI_QQVGA_30Hz;
  else
  {
    ROS_ERROR ("image resolution unknwon");
    exit (-1);
  }
  return -1;
}

void OpenNINode::mapMode (int resolution, XnMapOutputMode& output_mode) const
{
  switch (resolution)
  {
    case OpenNI_SXGA_15Hz: output_mode.nXRes = 1280;
      output_mode.nYRes = 1024;
      output_mode.nFPS = 15;
      break;
      break;
    case OpenNI_VGA_30Hz: output_mode.nXRes = 640;
      output_mode.nYRes = 480;
      output_mode.nFPS = 30;
      break;
      break;
    case OpenNI_QVGA_30Hz: output_mode.nXRes = 320;
      output_mode.nYRes = 240;
      output_mode.nFPS = 30;
      break;
      break;
    case OpenNI_QQVGA_30Hz: output_mode.nXRes = 160;
      output_mode.nYRes = 120;
      output_mode.nFPS = 30;
      break;
      break;
    default:
      ROS_ERROR ("image resolution unknwon");
      exit (-1);
      break;
  }
}

unsigned OpenNINode::getFPS (int mode) const
{
  switch (mode)
  {
    case OpenNI_SXGA_15Hz: return 15;
      break;
    case OpenNI_VGA_30Hz:
    case OpenNI_QVGA_30Hz:
    case OpenNI_QQVGA_30Hz:
      return 30;
      break;
    default:
      return 0;
      break;
  }
}

void OpenNINode::configCallback (Config &config, uint32_t level)
{
  /// @todo Think this is suppressing the software subsampled resolutions
  // check if image resolution is supported
  XnMapOutputMode output_mode, compatible_mode;
  mapMode (config.image_mode, output_mode);
  if (!device_->findFittingImageMode (output_mode, compatible_mode))
  {
    device_->getDefaultImageMode (compatible_mode);
    ROS_WARN ("Could not find any compatible image output mode %d x %d @ %d. "
              "Falling back to default mode %d x %d @ %d.",
              output_mode.nXRes, output_mode.nYRes, output_mode.nFPS,
              compatible_mode.nXRes, compatible_mode.nYRes, compatible_mode.nFPS);

    config.image_mode = mapMode (compatible_mode);
    image_height_ = compatible_mode.nYRes;
    image_width_ = compatible_mode.nXRes;
  }
  else
  { // we found a compatible mode => set image width and height as well as the mode of image stream
    image_height_ = output_mode.nYRes;
    image_width_ = output_mode.nXRes;
  }
  device_->setImageOutputMode (compatible_mode);


  mapMode (config.depth_mode, output_mode);
  if (!device_->findFittingImageMode (output_mode, compatible_mode))
  {
    device_->getDefaultDepthMode (compatible_mode);
    ROS_WARN ("Could not find any compatible depth output mode %d x %d @ %d. "
              "Falling back to default mode %d x %d @ %d.",
              output_mode.nXRes, output_mode.nYRes, output_mode.nFPS,
              compatible_mode.nXRes, compatible_mode.nYRes, compatible_mode.nFPS);

    // reset to compatible mode
    config.depth_mode = mapMode (compatible_mode);
    depth_width_ = compatible_mode.nXRes;
    depth_height_ = compatible_mode.nYRes;
  }
  else
  {
    depth_width_ = output_mode.nXRes;
    depth_height_ = output_mode.nYRes;
  }
  device_->setDepthOutputMode (compatible_mode);

  /// @todo Switching between registered and unregistered has weird behavior
#if 1
  bool do_registration = config_.point_cloud_type != OpenNI_XYZ_unregistered;
  device_->setDepthRegistration (do_registration);
#endif

  /// @todo Make sure in XYZRGB that image res is at least as large as depth res
  config_ = config;
}

void OpenNINode::run ()
{
  //while (comm_nh_.ok ());
  ros::spin();
}

} // namespace

using namespace openni_camera;

int main (int argc, char **argv)
{
  // Init ROS
  init (argc, argv, "openni_node");

  NodeHandle comm_nh ("openni_camera"); // for topics, services
  NodeHandle param_nh ("~"); // for parameters
  string deviceID = "";
  param_nh.getParam ("deviceID", deviceID);
  string topic = "";
  param_nh.getParam ("topic", topic);

  OpenNIDriver& driver = OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () == 0)
  {
    ROS_ERROR ("No devices connected.");
    exit (-1);
  }
  ROS_INFO ("Number devices connected: %d", driver.getNumberDevices ());

  boost::shared_ptr<OpenNIDevice> device;
  if (deviceID == "")
  {
    ROS_WARN ("%s deviceID is not set! Using first device.", argv[0]);
    device = driver.getDeviceByIndex (0);
  }
  else
  {
    if (deviceID.find ('@') != string::npos)
    {
      cout << "search by address" << endl;
      size_t pos = deviceID.find ('@');
      unsigned bus = atoi (deviceID.substr (0, pos).c_str ());
      unsigned address = atoi (deviceID.substr (pos + 1, deviceID.length () - pos - 1).c_str ());
      ROS_INFO ("searching for device with bus@address = %d@%d", bus, address);
      device = driver.getDeviceByAddress (bus, address);
    }
    else if (deviceID.length () > 2)
    {
      cout << "search by serial number" << endl;
      ROS_INFO ("searching for device with serial number = %s", deviceID.c_str ());
      device = driver.getDeviceBySerialNumber (deviceID);
    }
    else
    {
      cout << "search by index" << endl;
      unsigned index = atoi (deviceID.c_str ());
      ROS_INFO ("searching for device with index = %d", index);
      device = driver.getDeviceByIndex (index);
    }
  }
  
  if (!device)
  {
    ROS_ERROR ("%s No matching device found.", argv[0]);
    exit (-1);
  }
  else
  {
    unsigned short vendor_id, product_id;
    unsigned char bus, address;
    device->getDeviceInfo (vendor_id, product_id, bus, address);

    string vendor_name = "unknown";
    if (vendor_id == 0x1d27)
      vendor_name = "Primesense";
    else if (vendor_id == 0x45e)
      vendor_name = "Kinect";

    ROS_INFO ("Opened a %s device on bus %d:%d with serial number %s",
              vendor_name.c_str (), bus, address, device->getSerialNumber ());
  }

  OpenNINode openni_node (comm_nh, param_nh, device, topic);
  openni_node.run ();
    
  return (0);
}
