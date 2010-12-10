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
#include "openni_camera/openni_nodelets.h"

typedef openni_camera::OpenNIDriverNodelet OpenNIDriver;

PLUGINLIB_DECLARE_CLASS (openni_camera, OpenNIDriver, OpenNIDriver, nodelet::Nodelet);

void
openni_camera::OpenNIDriverNodelet::onInit ()
{
  /// @todo What exactly goes on with the threading here? -PM
  ros::NodeHandle comm_nh( getMTNodeHandle().resolveName("camera") ); // for topics, services
  ros::NodeHandle param_nh = getMTPrivateNodeHandle (); // for parameters
  driver_ = new OpenNIDriver (comm_nh, param_nh);

  int device_id;
  param_nh.param ("device_id", device_id, 0);

  if (!driver_->init (device_id))
    return;
  if (!driver_->start ())
    return;

  spinthread_ = new boost::thread (boost::bind (&OpenNIDriverNodelet::spin, this));
}

void
openni_camera::OpenNIDriverNodelet::spin ()
{
  driver_->spin ();
}

