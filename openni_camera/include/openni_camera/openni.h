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
#include <stereo_msgs/DisparityImage.h>

#include <ros/ros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <openni_camera/OpenNIConfig.h>

#include <XnOS.h>
#include <XnCppWrapper.h>

#define AVG(a,b) (((int)(a) + (int)(b)) >> 1)
#define AVG3(a,b,c) (((int)(a) + (int)(b) + (int)(c)) / 3)
#define AVG4(a,b,c,d) (((int)(a) + (int)(b) + (int)(c) + (int)(d)) >> 2)

namespace openni_camera
{
  class OpenNIDriver
  {
    public:
      /** \brief Constructor */
      OpenNIDriver (ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
      
      /** \brief Destructor */
      virtual ~OpenNIDriver ();
      
      /** \brief Spin (!)
        */
      bool spin ();

    protected:
      /** \brief Process raw depth data into ROS messages. */
      void processDepth ();

      /** \brief Process raw RGB data into ROS messages. */
      void processRgb ();

      /** \brief Process raw depth data into image message and publish. */
      void publishDepthImage ( const xn::DepthMetaData& depth_md, ros::Time time );

      /** \brief Process unregistered depth data into DisparityImage message and publish. */
      void publishDisparity ( const xn::DepthMetaData& depth_md );

      /** \brief Process unregistered depth data into point cloud message and publish. */
      void publishUnregisteredPointCloud ( const xn::DepthMetaData& depth_md );

      /** \brief Process synchronized depth and color images into XYZRGB point cloud and publish. */
      void publishRegisteredPointCloud ( const sensor_msgs::ImageConstPtr& depth_msg,
                                         const sensor_msgs::ImageConstPtr& rgb_msg );
    
      /** \brief An OpenNI context object. */
      xn::Context context_;
      /** \brief Depth generator object. */
      xn::DepthGenerator depth_generator_;
      /** \brief Image generator object. */
      xn::ImageGenerator image_generator_;

   private:
      bool isRGBRequired() const;
      bool isGrayRequired() const;
      bool isImageStreamRequired() const;
      bool isDepthStreamRequired() const;
      
      void bayer2RGB ( const xn::ImageMetaData& bayer, sensor_msgs::Image& image, int method = 0 );
      void bayer2Gray ( const xn::ImageMetaData& bayer, sensor_msgs::Image& image, int method = 0 );
      //void disparity_downsampling( )
      /** \brief A copy of the communication NodeHandle. */
      ros::NodeHandle comm_nh_;
      ros::NodeHandle param_nh_;

      /** \brief ROS publishers. */
      image_transport::Publisher pub_bayer_;
      image_transport::CameraPublisher pub_rgb_, pub_gray_;
      image_transport::CameraPublisher pub_depth_image_;
      ros::Publisher pub_disparity_;
      ros::Publisher pub_depth_points2_;

      /** \brief Approximate synchronization for XYZRGB point clouds. */
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
      typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
      boost::shared_ptr<Synchronizer> depth_rgb_sync_;

      /** \brief Dynamic reconfigure. */
      typedef openni_camera::OpenNIConfig Config;
      typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
      ReconfigureServer reconfigure_server_;
      Config config_;

      XnDouble pixel_size_, baseline_;
      XnUInt64 focal_length_, shadow_value_, no_sample_value_, F_;

      /// @todo May actually want to allocate each time when using nodelets
      /** \brief Image data. */
      sensor_msgs::Image rgb_image_;
      /** \brief Image data. */
      sensor_msgs::Image gray_image_;
      /** \brief PointCloud2 data. */
      sensor_msgs::PointCloud2 cloud2_;
      /** \brief Camera info data. */
      sensor_msgs::CameraInfo depth_info_, rgb_info_;
      /** \brief Disparity Image */
      stereo_msgs::DisparityImage disp_image_;

      /** \brief Callback for dynamic_reconfigure */
      void configCb (Config &config, uint32_t level);

      bool updateDeviceSettings();

      static const double rgb_focal_length_;
  };
} // namespace openni_camera

#endif //OPENNI_NODE_OPENNI_H_
