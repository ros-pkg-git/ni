/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef OPENNI_VIEWER_NODELET_H_
#define OPENNI_VIEWER_NODELET_H_

#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <pcl_ros/subscriber.h>
#include <pcl_visualization/pcl_visualizer.h>

namespace openni_pcl
{
  ////////////////////////////////////////////////////////////////////////////////////////////
  class OpenNIViewerNodelet : public nodelet::Nodelet
  {
    protected:
      /** \brief Nodelet initialization routine. */
      virtual void onInit ();
  
      /** \brief Spin. */
      void spin (); 

      /** \brief PointCloud2 message callback. */
      void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud);

      /** \brief The ROS NodeHandle used for parameters, publish/subscribe, etc. */
      boost::shared_ptr<ros::NodeHandle> pnh_;

      /** \brief The input PointCloud2 subscriber. */
      pcl_ros::Subscriber<sensor_msgs::PointCloud2> sub_;

      /** \brief The PCLVisualizer object. */
      boost::shared_ptr<pcl_visualization::PCLVisualizer> viewer_;

      /** \brief Mutex. */
      boost::mutex mutex_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };
}

#endif  //#ifndef OPENNI_VIEWER_NODELET_H_
