/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011
 *    Suat Gedikli <gedikli@willowgarage.com>
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
#include <openni_camera/openni_driver.h>
#include <openni_camera/openni_device_kinect.h>
#include <openni_camera/openni_device_primesense.h>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <locale>
#include <cctype>
#include <usb.h>
#include <map>

using namespace std;

namespace openni_wrapper
{

OpenNIDriver::OpenNIDriver () throw (OpenNIException)
{
  // Initialize the Engine
  XnStatus status = context_.Init ();
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("initialization failed. Reason: %s", xnGetStatusString (status));

  updateDeviceList ();
}

void OpenNIDriver::updateDeviceList () throw (OpenNIException)
{
  // clear current list
  device_info_.clear ();
  depth_info_.clear ();
  image_info_.clear ();
  bus_map_.clear ();
  serial_map_.clear ();

  // enumerate all devices
  static xn::NodeInfoList node_info_list;
  XnStatus status = context_.EnumerateProductionTrees (XN_NODE_TYPE_DEVICE, NULL, node_info_list);
  if (status != XN_STATUS_OK && node_info_list.Begin () != node_info_list.End ())
    THROW_OPENNI_EXCEPTION ("enumerating devices failed. Reason: %s", xnGetStatusString (status));
  else if (node_info_list.Begin () == node_info_list.End ())
    THROW_OPENNI_EXCEPTION ("no compatible device found");

  for (xn::NodeInfoList::Iterator nodeIt = node_info_list.Begin (); nodeIt != node_info_list.End (); ++nodeIt)
  {
    device_info_.push_back (*nodeIt);
  }

  // create a depth generator for each device
  static xn::NodeInfoList depth_nodes;
  status = context_.EnumerateProductionTrees (XN_NODE_TYPE_DEPTH, NULL, depth_nodes, NULL);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("enumerating depth generators failed. Reason: %s", xnGetStatusString (status));

  for (xn::NodeInfoList::Iterator nodeIt = depth_nodes.Begin (); nodeIt != depth_nodes.End (); ++nodeIt)
  {
    depth_info_.push_back (*nodeIt);
  }

  // create image stream
  static xn::NodeInfoList image_nodes;
  //xn::Query query;
  //query.AddSupportedCapability()
  status = context_.EnumerateProductionTrees (XN_NODE_TYPE_IMAGE, NULL, image_nodes, NULL);
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("enumerating image generators failed. Reason: %s", xnGetStatusString (status));

  for (xn::NodeInfoList::Iterator nodeIt = image_nodes.Begin (); nodeIt != image_nodes.End (); ++nodeIt)
  {
    image_info_.push_back (*nodeIt);
  }

  // check if we have same number of streams as devices!
  if (device_info_.size () != depth_info_.size () || device_info_.size () != image_info_.size ())
    THROW_OPENNI_EXCEPTION ("number of streams and devices does not match: %d devices, %d depth streams, %d image streams", device_info_.size (), depth_info_.size (), image_info_.size ());

  // Hack to get serial numbers of devices. this is not done in the driver yet
  getDeviceInfo ();
}

void OpenNIDriver::stopAll () throw (OpenNIException)
{
  XnStatus status = context_.StopGeneratingAll ();
  if (status != XN_STATUS_OK)
    THROW_OPENNI_EXCEPTION ("stopping all streams failed. Reason: %s", xnGetStatusString (status));
}

OpenNIDriver::~OpenNIDriver () throw ()
{
  stopAll ();
  context_.Shutdown ();
}

boost::shared_ptr<OpenNIDevice> OpenNIDriver::createDevice (unsigned deviceIdx) const throw (OpenNIException)
{
  string connection_string = device_info_[deviceIdx].GetCreationInfo ();
  transform (connection_string.begin (), connection_string.end (), connection_string.begin (), std::towlower);
  if (connection_string.substr (0, 4) == "045e")
  {
    DeviceKinect* device = new DeviceKinect (context_, device_info_[deviceIdx], image_info_[deviceIdx], depth_info_[deviceIdx]);
    return boost::shared_ptr<OpenNIDevice > (device);
  }
  else if (connection_string.substr (0, 4) == "1d27")
  {
    DevicePrimesense* device = new DevicePrimesense (context_, device_info_[deviceIdx], image_info_[deviceIdx], depth_info_[deviceIdx]);
    return boost::shared_ptr<OpenNIDevice > (device);
  }
  else
    THROW_OPENNI_EXCEPTION ("vecndor %s known by primesense driver, but not by ros driver. Contact maintainer of the ros driver.");

  return boost::shared_ptr<OpenNIDevice > ((OpenNIDevice*)NULL);
}

boost::shared_ptr<OpenNIDevice> OpenNIDriver::getDeviceByIndex (unsigned index) const throw (OpenNIException)
{
  return createDevice (index);
}

boost::shared_ptr<OpenNIDevice> OpenNIDriver::getDeviceBySerialNumber (const string& serial_number) const throw (OpenNIException)
{
  map<string, int>::const_iterator it = serial_map_.find (serial_number);

  if (it != serial_map_.end () && it->second >= 0)
  {
    return createDevice (it->second);
  }

  return boost::shared_ptr<OpenNIDevice > ((OpenNIDevice*)NULL);
}

boost::shared_ptr<OpenNIDevice> OpenNIDriver::getDeviceByAddress (unsigned char bus, unsigned char address) const throw (OpenNIException)
{
  map<unsigned char, map<unsigned char, int> >::const_iterator busIt = bus_map_.find (bus);
  if (busIt != bus_map_.end ())
  {
    map<unsigned char, int>::const_iterator devIt = busIt->second.find (address);
    if (devIt != busIt->second.end () && devIt->second >= 0)
    {
      return createDevice (devIt->second);
    }
  }

  return boost::shared_ptr<OpenNIDevice > ((OpenNIDevice*)NULL);
}

void OpenNIDriver::getDeviceInfo ()
{
  // extract bus and path for each of our devices -> unique
  //map< unsigned char, map< unsigned char, unsigned > > device_map;
  for (unsigned deviceIdx = 0; deviceIdx < device_info_.size (); ++deviceIdx)
  {
    XnUInt16 nVendorID = 0;
    XnUInt16 nProductID = 0;
    XnUInt8 nBus = 0;
    XnUInt8 nAddress = 0;
    sscanf (device_info_[deviceIdx].GetCreationInfo (), "%hx/%hx@%hhu/%hhu", &nVendorID, &nProductID, &nBus, &nAddress);

    bus_map_[ nBus ][ nAddress ] = deviceIdx;
  }

  struct usb_bus *bus;
  struct usb_device *dev;

  /* Initialize libusb */
  usb_init ();

  /* Find all USB busses on system */
  usb_find_busses ();

  /* Find all devices on all USB devices */
  usb_find_devices ();

  for (bus = usb_busses; bus; bus = bus->next)
  {
    unsigned char busId = atoi (bus->dirname);
    if (bus_map_.find (busId) == bus_map_.end ())
      continue;

    for (dev = bus->devices; dev; dev = dev->next)
    {
      unsigned char devId = atoi (dev->filename);
      if (bus_map_[busId].find (devId) == bus_map_[busId].end ())
        continue;

      unsigned nodeIdx = bus_map_[busId][devId];
      xn::NodeInfo& current_node = device_info_[nodeIdx];
      XnProductionNodeDescription& description = const_cast<XnProductionNodeDescription&> ( current_node.GetDescription() );

      int ret;
      char buffer[256];
      usb_dev_handle *udev;

      /* Opens a USB device */
      udev = usb_open (dev);
      if (udev)
      {
        if (dev->descriptor.iManufacturer)
        {
          // Retrieves a string descriptor from a device using the first language
          ret = usb_get_string_simple(udev, dev->descriptor.iManufacturer, buffer, sizeof (buffer));
          if (ret > 0)
            strcpy( description.strVendor, buffer );
          else
            strcpy( description.strVendor, "unknwon" );
        }

        if (dev->descriptor.iProduct)
        {
          ret = usb_get_string_simple(udev, dev->descriptor.iProduct, buffer, sizeof (buffer));
          if (ret > 0)
            strcpy( description.strName, buffer );
          else
            strcpy( description.strName, "unknwon" );
        }
         
        if (dev->descriptor.iSerialNumber)
        {
          ret = usb_get_string_simple (udev, dev->descriptor.iSerialNumber, buffer, sizeof (buffer));
          if (ret > 0)
          {
            current_node.SetInstanceName (buffer);
            string serial = buffer;

            if (serial_map_.find (serial) == serial_map_.end ())
            {
              serial_map_[serial] = nodeIdx;
            }
            else // collision -> set to -1 since non-unique
            {
              serial_map_[serial] = -1;
            }
          }
          else
            current_node.SetInstanceName ("");
        }
        else
          current_node.SetInstanceName ("");

        /* Closes a USB device */
        usb_close (udev);
      }
    }
  }
}
} // namespace
