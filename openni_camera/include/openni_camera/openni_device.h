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

#ifndef __OPENNI_IDEVICE_H__
#define __OPENNI_IDEVICE_H__
#include <map>
#include <vector>
#include <utility>
#include <openni_camera/openni_exception.h>
#include <openni_camera/openni_image.h>
#include <openni_camera/openni_depth_image.h>
#include <XnCppWrapper.h>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

/// @todo Get rid of all exception-specifications, these are useless and soon to be deprecated

namespace openni_wrapper
{
/**
 * @brief Class representing an astract device for Primesense or MS Kinect devices.
 * @author Suat Gedikli
 * @date 02.january 2011
 */
class OpenNIDevice : public boost::noncopyable
{
public:
  typedef boost::function<void(const Image&, void* cookie) > ImageCallbackFunction;
  typedef boost::function<void(const DepthImage&, void* cookie) > DepthImageCallbackFunction;
  typedef unsigned CallbackHandle;
public:
  virtual ~OpenNIDevice () throw ();

  virtual bool findFittingImageMode (const XnMapOutputMode& output_mode, XnMapOutputMode& mode ) const throw (OpenNIException);
  virtual bool findFittingDepthMode (const XnMapOutputMode& output_mode, XnMapOutputMode& mode ) const throw (OpenNIException);

  virtual bool isImageModeSupported (const XnMapOutputMode& output_mode) const throw (OpenNIException);
  virtual bool isDepthModeSupported (const XnMapOutputMode& output_mode) const throw (OpenNIException);

  virtual void getDefaultImageMode (XnMapOutputMode& output_mode) const throw ();
  virtual void getDefaultDepthMode (XnMapOutputMode& output_mode) const throw ();

  virtual void setImageOutputMode (const XnMapOutputMode& output_mode) throw (OpenNIException);
  virtual void setDepthOutputMode (const XnMapOutputMode& output_mode) throw (OpenNIException);

  void getImageOutputMode (XnMapOutputMode& output_mode) const throw (OpenNIException);
  void getDepthOutputMode (XnMapOutputMode& output_mode) const throw (OpenNIException);

  void setDepthRegistration (bool on_off) throw (OpenNIException);
  bool isDepthRegistered () const throw (OpenNIException);

  /** \brief returns the focal length for the color camera in pixels. pixels are assumed to be square.
   Result depends on the output resolution of the image.
   */
  inline float getImageFocalLength (int output_x_resolution = 0) const throw ();
  /** \brief returns the focal length for the ir camera in pixels. pixels are assumed to be square.
   Result depends on the output resolution of the depth image.
   */
  inline float getDepthFocalLength (int output_x_resolution = 0) const throw ();
  inline float getBaseline () const throw ();

  virtual void startImageStream () throw (OpenNIException);
  virtual void stopImageStream () throw (OpenNIException);

  virtual void startDepthStream () throw (OpenNIException);
  virtual void stopDepthStream () throw (OpenNIException);

  bool isImageRunning () const throw (OpenNIException);
  bool isDepthRunning () const throw (OpenNIException);

  CallbackHandle registerImageCallback (const ImageCallbackFunction& callback, void* cookie) throw ();
  template<typename T> CallbackHandle registerImageCallback (void (T::*callback)(const Image&, void* cookie), T& instance, void* cookie) throw ();
  bool unregisterImageCallback (const CallbackHandle& callbackHandle) throw ();

  CallbackHandle registerDepthCallback (const DepthImageCallbackFunction& callback, void* cookie) throw ();
  template<typename T> CallbackHandle registerDepthCallback (void (T::*callback)(const DepthImage&, void* cookie), T& instance, void* cookie) throw ();
  bool unregisterDepthCallback (const CallbackHandle& callbackHandle) throw ();

  /** \brief returns the serial number for device.
   *  \attention This might be an empty string!!!
   */
  const char* getSerialNumber () const throw ();
  /** \brief returns the connectionstring for current device, which has following format vendorID/productID@BusID/DeviceID */
  const char* getConnectionString () const throw ();
  /** \brief get more information aout the device*/
  void getDeviceInfo (unsigned short& vendor_id, unsigned short& product_id, unsigned char& bus, unsigned char& address)const throw ();
protected:
  OpenNIDevice (xn::Context& context, const xn::NodeInfo& device_node, const xn::NodeInfo& image_node, const xn::NodeInfo& depth_node) throw (OpenNIException);
  static void NewDepthDataAvailable (xn::ProductionNode& node, void* cookie) throw ();
  static void NewImageDataAvailable (xn::ProductionNode& node, void* cookie) throw ();

  // This is a workaround, since in the NewDepthDataAvailable function WaitAndUpdateData leads to a dead-lock behaviour
  // and retrieving image data without WaitAndUpdateData leads to incomplete images!!!
  void ImageDataThreadFunction () throw (OpenNIException);
  void DepthDataThreadFunction () throw (OpenNIException);
  //static void NewImageDataAvailable ( xn::ProductionNode& node, void* cookie );

  virtual bool isImageResizeSupported (unsigned input_width, unsigned input_height, unsigned output_width, unsigned output_height) const  throw () = 0;

  void setRegistration (bool on_off) throw (OpenNIException);
  virtual Image* getCurrentImage (const xn::ImageMetaData& image_data) const throw () = 0;

  virtual void getAvailableModes () throw (OpenNIException);
  void Init () throw (OpenNIException);
  // holds the callback functions together with custom data
  // since same callback function can be registered multiple times with e.g. different custom data
  // we use a map structure with a handle as the key
  std::map< CallbackHandle, std::pair<ImageCallbackFunction, void*> > image_callback_;
  std::map< CallbackHandle, std::pair<DepthImageCallbackFunction, void*> > depth_callback_;

  std::vector<XnMapOutputMode> available_image_modes_;
  std::vector<XnMapOutputMode> available_depth_modes_;

  /** \brief node object for current device */
  const xn::NodeInfo& device_node_info_;
  /** \brief Depth generator object. */
  xn::DepthGenerator depth_generator_;
  /** \brief Image generator object. */
  xn::ImageGenerator image_generator_;

  XnCallbackHandle depth_callback_handle_;
  XnCallbackHandle image_callback_handle_;

  /** \brief focal length for IR camera producing depth information in native SXGA mode */
  float depth_focal_length_SXGA_;
  /** \brief distance between the projector and the IR camera*/
  float baseline_;
  /** \brief focal length for regular camera producing color images in native SXGA mode*/
  static const float rgb_focal_length_SXGA_;

  xn::Context& context_;
  /** the value for shadow (occluded pixels) */
  XnUInt64 shadow_value_;
  /** the value for pixels without a valid disparity measurement */
  XnUInt64 no_sample_value_;

  OpenNIDevice::CallbackHandle image_callback_handle_counter_;
  OpenNIDevice::CallbackHandle depth_callback_handle_counter_;

  boost::thread image_thread_;
  boost::thread depth_thread_;
  boost::condition_variable image_condition_;
  boost::condition_variable depth_condition_;
  boost::mutex image_mutex_;
  boost::mutex depth_mutex_;
  bool running_;
};

float OpenNIDevice::getImageFocalLength (int output_x_resolution) const throw ()
{
  if (output_x_resolution == 0)
  {
    XnMapOutputMode output_mode;
    getImageOutputMode (output_mode);
    output_x_resolution = output_mode.nXRes;
  }
  float scale = 1280.0f / output_x_resolution;
  return rgb_focal_length_SXGA_ * scale;
}

float OpenNIDevice::getDepthFocalLength (int output_x_resolution) const throw ()
{
  if (output_x_resolution == 0)
  {
    XnMapOutputMode output_mode;
    getDepthOutputMode (output_mode);
    output_x_resolution = output_mode.nXRes;
  }
  float scale = 1280.0f / output_x_resolution;
  if (isDepthRegistered ())
    return rgb_focal_length_SXGA_ * scale;
  else
    return depth_focal_length_SXGA_ * scale;
}

float OpenNIDevice::getBaseline () const throw ()
{
  return baseline_;
}

template<typename T> OpenNIDevice::CallbackHandle OpenNIDevice::registerImageCallback (void (T::*callback)(const Image&, void* cookie), T& instance, void* custom_data) throw ()
{
  ImageCallbackFunction callback_function = boost::bind (callback, boost::ref (instance), _1, _2);
  image_callback_[image_callback_handle_counter_] = std::make_pair (callback_function, custom_data);
  return image_callback_handle_counter_++;
}

template<typename T> OpenNIDevice::CallbackHandle OpenNIDevice::registerDepthCallback (void (T::*callback)(const DepthImage&, void* cookie), T& instance, void* custom_data) throw ()
{
  DepthImageCallbackFunction callback_function = boost::bind (callback, boost::ref (instance), _1, _2);
  depth_callback_[depth_callback_handle_counter_] = std::make_pair (callback_function, custom_data);
  return depth_callback_handle_counter_++;
}
}
#endif // __OPENNI_IDEVICE_H__
