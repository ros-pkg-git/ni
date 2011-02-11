/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
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
#ifndef OPENNI_NODELET_OPENNI_H_
#define OPENNI_NODELET_OPENNI_H_

#include <nodelet/nodelet.h>
#include "openni_camera/openni_driver.h"
#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <openni_camera/OpenNIConfig.h>
#include <image_transport/image_transport.h>
#include <string>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Core>

namespace openni_camera
{
  ////////////////////////////////////////////////////////////////////////////////////////////
  class OpenNINodelet : public nodelet::Nodelet
  {
    public:
      virtual ~OpenNINodelet ();
    private:
      typedef OpenNIConfig Config;
      typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
      typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

      /** \brief Nodelet initialization routine. */
      virtual void onInit ();
      void setupDevice (ros::NodeHandle& param_nh);
      void updateModeMaps ();
      void startSynchronization ();
      void stopSynchronization ();
      void setupDeviceModes (int image_mode, int depth_mode);
      bool isImageModeSupported (int image_mode) const;
      bool isDepthModeSupported (int depth_mode) const;

      void configCallback (Config &config, uint32_t level);
      int mapXnMode2ConfigMode (const XnMapOutputMode& output_mode) const;
      XnMapOutputMode mapConfigMode2XnMode (int mode) const;

      // callback methods
      void imageCallback (const openni_wrapper::Image& image, void* cookie);
      void depthCallback (const openni_wrapper::DepthImage& depth_image, void* cookie);
      void subscriberChangedEvent ();

      // helper methods
      inline bool isImageStreamRequired() const;
      inline bool isDepthStreamRequired() const;
      sensor_msgs::CameraInfoPtr fillCameraInfo (ros::Time time, bool is_rgb);

      // published topics
      ros::Publisher pub_rgb_info_, pub_depth_info_;
      image_transport::Publisher pub_rgb_image_, pub_gray_image_, pub_depth_image_;
      ros::Publisher pub_disparity_;
      ros::Publisher pub_point_cloud_;
      ros::Publisher pub_point_cloud_rgb_;

      // Approximate synchronization for XYZRGB point clouds.
      boost::shared_ptr<Synchronizer> depth_rgb_sync_;

      // publish methods
      void publishRgbImage (const openni_wrapper::Image& image, ros::Time time) const;
      void publishGrayImage (const openni_wrapper::Image& image, ros::Time time) const;
      void publishDepthImage (const openni_wrapper::DepthImage& depth, ros::Time time) const;
      void publishDisparity (const openni_wrapper::DepthImage& depth, ros::Time time) const;
      void publishXYZPointCloud (const openni_wrapper::DepthImage& depth, ros::Time time) const;
      void publishXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg) const;

      /** \brief the actual openni device*/
      boost::shared_ptr<openni_wrapper::OpenNIDevice> device_;

      /** \brief reconfigure server*/
      boost::shared_ptr<ReconfigureServer> reconfigure_server_;
      Config config_;
      boost::recursive_mutex reconfigure_mutex_;
      //boost::mutex depth_mutex_;
      //boost::mutex image_mutex_;

      std::string rgb_frame_id_;
      std::string depth_frame_id_;
      unsigned image_width_;
      unsigned image_height_;
      unsigned depth_width_;
      unsigned depth_height_;

      struct modeComp
      {
        bool operator () (const XnMapOutputMode& mode1, const XnMapOutputMode& mode2) const
        {
          if (mode1.nXRes < mode2.nXRes)
            return true;
          else if (mode1.nXRes > mode2.nXRes)
            return false;
          else if (mode1.nYRes < mode2.nYRes)
            return true;
          else if (mode1.nYRes > mode2.nYRes)
            return false;
          else if (mode1.nFPS < mode2.nFPS)
            return true;
          else
            return false;
        }
      };
      std::map<XnMapOutputMode, int, modeComp> xn2config_map_;
      std::map<int, XnMapOutputMode> config2xn_map_;
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  bool OpenNINodelet::isImageStreamRequired() const
  {
    return (pub_rgb_image_.getNumSubscribers()       > 0 ||
            pub_gray_image_.getNumSubscribers()      > 0 ||
            pub_point_cloud_rgb_.getNumSubscribers() > 0 );
  }

  bool OpenNINodelet::isDepthStreamRequired() const
  {
    return (pub_depth_image_.getNumSubscribers()     > 0 ||
            pub_disparity_.getNumSubscribers()       > 0 ||
            pub_point_cloud_.getNumSubscribers()     > 0 ||
            pub_point_cloud_rgb_.getNumSubscribers() > 0 );
  }

}

#endif  //#ifndef OPENNI_NODELET_OPENNI_H_
