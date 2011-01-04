/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010
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

#ifndef OPENNI_OPENNI_H_
#define OPENNI_OPENNI_H_
#include <string>
#include <vector>
#include <map>
#include <openni_camera/openni_exception.h>
#include <boost/shared_ptr.hpp>
#include <XnCppWrapper.h>

namespace openni_wrapper
{
class OpenNIDevice;
/**
 * @brief Driver class implemented as Singleton. This class contains the xn::Context object used by all devices. It \
 * provides methods for enumerating and accessing devices.
 * @author Suat Gedikli
 * @date 02.january 2011
 */
class OpenNIDriver
{
public:
  inline static OpenNIDriver& getInstance () throw (OpenNIException);
  inline unsigned getNumberDevices () const throw ();
  inline const std::vector<xn::NodeInfo>& getDeviceInfoList () const throw ();
  void updateDeviceList () throw (OpenNIException);
  boost::shared_ptr<OpenNIDevice> getDeviceByIndex (unsigned index) const throw (OpenNIException);
  boost::shared_ptr<OpenNIDevice> getDeviceBySerialNumber (const std::string& serial_number) const throw (OpenNIException);
  boost::shared_ptr<OpenNIDevice> getDeviceByAddress (unsigned char bus, unsigned char address) const throw (OpenNIException);

  void stopAll () throw (OpenNIException);
  ~OpenNIDriver () throw ();
protected:
  OpenNIDriver () throw (OpenNIException);
  boost::shared_ptr<OpenNIDevice> createDevice (unsigned index) const throw (OpenNIException);

  // workaround until Primesense Sensor fix this
  void getDeviceInfo ();
  std::vector<xn::NodeInfo> device_info_;
  std::vector<xn::NodeInfo> depth_info_;
  std::vector<xn::NodeInfo> image_info_;
  mutable xn::Context context_;

  std::map< unsigned char, std::map<unsigned char, int > > bus_map_;
  std::map< std::string, int > serial_map_;
};

OpenNIDriver& OpenNIDriver::getInstance () throw (OpenNIException)
{
  static OpenNIDriver driver;
  return driver;
}

unsigned OpenNIDriver::getNumberDevices () const throw ()
{
  return (unsigned)device_info_.size ();
}

const std::vector<xn::NodeInfo>& OpenNIDriver::getDeviceInfoList () const throw ()
{
  return device_info_;
}
} // namespace
#endif