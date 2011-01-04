#include "openni_camera/openni_exception.h"
#include <sstream>

namespace openni_wrapper
{

OpenNIException::OpenNIException (const std::string& function_name, const std::string& file_name, unsigned line_number, const std::string& message) throw ()
: function_name_ (function_name)
, file_name_ (file_name)
, line_number_ (line_number)
, message_ (message)
{
  std::stringstream sstream;
  sstream << function_name_ << " @ " << file_name_ << " @ " << line_number_ << " : " << message_;
  message_long_ = sstream.str();
}

OpenNIException::~OpenNIException () throw ()
{
}

OpenNIException& OpenNIException::operator= (const OpenNIException& exception) throw ()
{
  message_ = exception.message_;
}

const char* OpenNIException::what () const throw ()
{
  return message_long_.c_str();
}

const std::string& OpenNIException::getFunctionName () const throw ()
{
  return function_name_;
}

const std::string& OpenNIException::getFileName () const throw ()
{
  return file_name_;
}

unsigned OpenNIException::getLineNumber () const throw ()
{
  return line_number_;
}

} //namespace openni_camera