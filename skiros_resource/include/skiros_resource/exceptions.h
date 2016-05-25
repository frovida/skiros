#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#include <stdexcept>
//#include "skiros_common/logger_sys.h"
#include <ros/ros.h>
namespace skiros_device
{
		class EndEffectorException: public std::runtime_error
		{
		  public:
		  explicit EndEffectorException(const std::string& what_arg):
		  std::runtime_error(what_arg)
		  {
			ROS_ERROR("End effector exception: %s", what_arg.c_str());
		  }
		};
}
#endif // EXCEPTIONS_H
