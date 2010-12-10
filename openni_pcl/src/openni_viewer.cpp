/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: pointcloud_online_viewer.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

// ROS core
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
// PCL includes
#include <pcl/point_types.h>
#include <pcl_visualization/pcl_visualizer.h>

sensor_msgs::PointCloud2ConstPtr cloud_, cloud_old_;
boost::mutex m;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  m.lock ();
  cloud_ = cloud;
  m.unlock ();
}

/* ---[ */
int
main (int argc, char** argv)
{
  ros::init (argc, argv, "openni_viewer");
  ros::NodeHandle nh ("~");

  // Create a ROS subscriber
  ros::Subscriber sub = nh.subscribe ("input", 15, cloud_cb);

  ROS_INFO ("Subscribing to %s for PointCloud2 messages...", nh.resolveName ("input").c_str ());

  pcl_visualization::PCLVisualizer p (argc, argv, "OpenNI Kinect Viewer");
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl_visualization::PointCloudColorHandler<sensor_msgs::PointCloud2>::Ptr color_handler;

  double psize = 0;
  while (nh.ok ())
  {
    // Spin
    ros::spinOnce ();
    ros::Duration (0.0001).sleep ();
    p.spinOnce (10);

    // If no cloud received yet, continue
    if (!cloud_)
      continue;

    if (cloud_ == cloud_old_)
      continue;

    // Save the last point size used
    p.getPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud");

    p.removePointCloud ("cloud");
    // Convert to PointCloud<T>
    m.lock ();
    pcl::fromROSMsg (*cloud_, cloud_xyz);
 
    color_handler.reset (new pcl_visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (*cloud_));
    p.addPointCloud (cloud_xyz, color_handler, "cloud");

    // Set the point size
    p.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, psize, "cloud");

    cloud_old_ = cloud_;
    m.unlock ();
  }

  return (0);
}
/* ]--- */
