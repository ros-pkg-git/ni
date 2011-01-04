#ifndef __OPENNI_EXCEPTION__
#define __OPENNI_EXCEPTION__

#include <cstdarg>
#include <cstdio>
#include <exception>
#include <string>

#define THROW_OPENNI_EXCEPTION(format, args...) throwOpenNIException( __PRETTY_FUNCTION__, __FILE__, __LINE__, format , ##args )


namespace openni_wrapper
{
/**
 * @brief General exception class
 * @author Suat Gedikli
 * @date 02.january 2011
 */
class OpenNIException : public std::exception
{
public:
  OpenNIException (const std::string& function_name, const std::string& file_name, unsigned line_number, const std::string& message) throw ();
  virtual ~OpenNIException () throw ();
  OpenNIException & operator= (const OpenNIException& exception) throw ();
  virtual const char* what () const throw ();

  const std::string& getFunctionName () const throw ();
  const std::string& getFileName () const throw ();
  unsigned getLineNumber () const throw ();
protected:
  std::string function_name_;
  std::string file_name_;
  unsigned line_number_;
  std::string message_;
  std::string message_long_;
};

inline void throwOpenNIException (const char* function, const char* file, unsigned line, const char* format, ...)
{
  static char msg[1024];
  va_list args;
  va_start (args, format);
  vsprintf (msg, format, args);
  throw OpenNIException (function, file, line, msg);
}
} // namespace openni_camera
#endif