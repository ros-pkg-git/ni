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
#include <pluginlib/class_list_macros.h>
#include "openni_pcl/openni_viewer_nodelet.h"
#include <pcl/point_types.h>

typedef openni_pcl::OpenNIViewerNodelet OpenNIViewer;

PLUGINLIB_DECLARE_CLASS (openni_pcl, OpenNIViewer, OpenNIViewer, nodelet::Nodelet);

void
openni_pcl::OpenNIViewerNodelet::onInit ()
{
  pnh_.reset (new ros::NodeHandle (getMTPrivateNodeHandle ()));
  sub_.subscribe (*pnh_, "input", 1, bind (&OpenNIViewerNodelet::cloud_cb, this, _1));
  viewer_.reset (new pcl_visualization::PCLVisualizer ("OpenNI Kinect Viewer"));
  ROS_INFO ("[OpenNIViewer] Nodelet initialized.");
}

void
openni_pcl::OpenNIViewerNodelet::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  boost::mutex::scoped_lock lock (mutex_);

  // Save the last point size used
  double psize;
  //viewer_->getPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud");

  viewer_->removePointCloud ("cloud");
  
  // Convert to PointCloud<T>
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::fromROSMsg (*cloud, cloud_xyz);

  pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> h (*cloud);
  viewer_->addPointCloud (
      cloud_xyz, 
      boost::make_shared<pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> >(h), 
      "cloud");

  // Set the point size
  //viewer_->setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud");

  // Spin
  viewer_->spinOnce (10);
}

