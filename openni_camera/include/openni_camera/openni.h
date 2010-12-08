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

#ifndef OPENNI_NODE_OPENNI_H_
#define OPENNI_NODE_OPENNI_H_

#include <boost/thread/mutex.hpp>

// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <openni_camera/OpenNIConfig.h>

//@todo: warnings about deprecated header? 
#include <image_geometry/pinhole_camera_model.h>

#include <Eigen/Core>

#include <XnOS.h>
#include <XnCppWrapper.h>

namespace openni_camera
{
  class OpenNIDriver
  {
    public:
      /** \brief Constructor */
      OpenNIDriver (ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
      virtual ~OpenNIDriver ();

      /** \brief Start (resume) the data acquisition process. */
      bool start ();
      /** \brief Stop (pause) the data acquisition process. */
      void stop ();

      /** \brief Initialize a OpenNI device, given an index.
        * \param index the index of the device to initialize
        */
      bool init (int index);

      /** \brief Spin (!)
        */
      bool spin ();

    protected:
      /** \brief Send the data over the network. */
      void publish ();

      void publishImu();

      /** \brief An OpenNI context object. */
      xn::Context context_;

      /** \brief Return code for any OpenNI call. */
      XnStatus rc_;

      /** \brief Depth generator object. */
      xn::DepthGenerator depth_;
      /** \brief Image generator object. */
      xn::ImageGenerator image_;
      /** \brief Depth meta data. */
      xn::DepthMetaData depth_md_;
      /** \brief Image meta data. */
      xn::ImageMetaData image_md_;

   private:
      /** \brief A copy of the communication NodeHandle. */
      ros::NodeHandle comm_nh_;

      /** \brief Internal mutex. */
      boost::mutex buffer_mutex_;

      /** \brief ROS publishers. */
      image_transport::CameraPublisher pub_rgb_, pub_depth_, pub_ir_;
      image_transport::Publisher pub_rgb_rect_;
      ros::Publisher pub_depth_points_, pub_depth_points2_;
      ros::Publisher pub_rgb_points2_;
      ros::Publisher pub_imu_;

      /** \brief Camera info manager objects. */
      boost::shared_ptr<CameraInfoManager> rgb_info_manager_, depth_info_manager_;
      image_geometry::PinholeCameraModel rgb_model_, depth_model_;

      /** \brief Dynamic reconfigure. */
      typedef openni_camera::OpenNIConfig Config;
      typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
      ReconfigureServer reconfigure_server_;
      Config config_;

      /** \brief Camera parameters. */
      int width_;
      int height_;
      double shift_offset_;
      //double baseline_; // between IR projector and depth camera
      Eigen::Matrix<double, 3, 4> depth_to_rgb_;
      static const double SHIFT_SCALE;
      
      /** \brief True if we're acquiring images. */
      bool started_;

      XnDouble pixel_size_, baseline_;
      XnUInt64 focal_length_, shadow_value_, no_sample_value_, F_;

      /** \brief Pointer to the RGB buffer data. */
      const XnUInt8 *rgb_buf_;
      //const XnRGB24Pixel* rgb_buf_;
      /** \brief Pointer to the depth buffer data. */
      const XnDepthPixel* depth_buf_;

      /// @todo May actually want to allocate each time when using nodelets
      /** \brief Image data. */
      sensor_msgs::Image rgb_image_, depth_image_;
      /** \brief PointCloud data. */
      sensor_msgs::PointCloud cloud_;
      /** \brief PointCloud2 data. */
      sensor_msgs::PointCloud2 cloud2_;
      /** \brief Camera info data. */
      sensor_msgs::CameraInfo rgb_info_, depth_info_;
      /** \brief Accelerometer data. */
      sensor_msgs::Imu imu_msg_;

      /** \brief Accelerometer data */
      double accel_x_, accel_y_, accel_z_;

      /** \brief Tilt sensor */
      double tilt_angle_; // [deg]

      /** \brief Callback for dynamic_reconfigure */
      void configCb (Config &config, uint32_t level);

      void updateDeviceSettings();
    
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

} // namespace openni_camera

#endif //OPENNI_NODE_OPENNI_H_
