/***************************************************************************
 * Software License Agreement (BSD License)                                *
 *                                                                         *
 *  Copyright (c) 2012, Christian Caroe, Casper Schou                      *
 *	Department of Mechanical and Manufacturing Engineering                 *
 *  Aalborg University, Denmark                                            *
 *  All rights reserved.                                                   *
 *                                                                         *
 *  Redistribution and use in source and binary forms, with or without     *
 *  modification, are permitted provided that the following conditions     *
 *  are met:                                                               *
 *                                                                         *
 *  - Redistributions of source code must retain the above copyright       *
 *     notice, this list of conditions and the following disclaimer.       *
 *  - Redistributions in binary form must reproduce the above              *
 *     copyright notice, this list of conditions and the following         *
 *     disclaimer in the documentation and/or other materials provided     *
 *     with the distribution.                                              *
 *  - Neither the name of Aalborg University nor the names of              *
 *     its contributors may be used to endorse or promote products derived *
 *     from this software without specific prior written permission.       *
 *                                                                         *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    *
 *  'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS      *
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE         *
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,    *
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,   *
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;       *
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER       *
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT     *
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN      *
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE        *
 *  POSSIBILITY OF SUCH DAMAGE.                                            *
 ***************************************************************************
 *
 */

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <ctime>
#include "log4cxx/rollingfileappender.h"
#include "log4cxx/patternlayout.h"
#include "skiros_common/logger_sys.h"
#include "skiros_common/utility.h"
#include "ros/ros.h"
#include "ros/console.h"

namespace skiros_common
{
	//Define log-file information

	bool debug_activated, info_activated, warn_activated;
	bool ros_output = true;

	log4cxx::LoggerPtr debug_logger, info_logger, warn_logger;

	log4cxx::spi::LocationInfo MyLocation;

	std::string findLogPath()
    {
        std::string path = utility::getSkirosSaveDirectory();
        path = path + "logs/";
		return path;
	}

	void toggleOutput()
	{
		if(ros_output) ros_output = false;
		else ros_output = true;
	}

	void InitLogger(std::string package_name, bool debugON, bool infoON, bool warnON)
	{
		std::string log_dir = findLogPath();

		debug_activated = false;
		info_activated = false;
		warn_activated = false;

		//initiate logging system
		time_t current_time = time(NULL);
		struct tm * now = localtime( & current_time);
		std::stringstream time_date_stamp;
		time_date_stamp << (now->tm_year+1900) << '-'
				<< (now->tm_mon +1) << '-'
				<< (now->tm_mday) << '_'
				<< now->tm_hour << ':'
				<< now->tm_min << ':'
				<< now->tm_sec;

		std::stringstream log_file_debug, log_file_info, log_file_warn;
		log_file_debug << log_dir << package_name <<"/" << time_date_stamp.str() << "/" << package_name << "_" << time_date_stamp.str() <<"_debug.log";
		log_file_info << log_dir << package_name <<"/" << time_date_stamp.str() << "/" << package_name << "_" << time_date_stamp.str() <<"_info.log";
		log_file_warn << log_dir << package_name <<"/" << time_date_stamp.str() << "/" << package_name << "_" << time_date_stamp.str() <<"_warn.log";

		//Creating pointer to the logger (this will print all from default ros logger)
		std::stringstream logger_name_debug, logger_name_info, logger_name_warn;
		logger_name_debug << "debug_logger." << package_name;
		logger_name_info << "info_logger." << package_name;
		logger_name_warn << "warn_logger." << package_name;

		//Defining the layout of the logs
		log4cxx::LayoutPtr layout = new log4cxx::PatternLayout("[%d] [%20.20F(%4.4L)] : [%5.5p] %m\n");

		log4cxx::helpers::Pool pool;

		if(debugON)
		{
			debug_logger = log4cxx::Logger::getLogger(logger_name_debug.str());

			log4cxx::RollingFileAppenderPtr debug_appender = new log4cxx::RollingFileAppender(layout, log_file_debug.str(), false);

			debug_appender->setMaximumFileSize(100*1024*1024);
			debug_appender->setMaxBackupIndex(10);
			debug_appender->activateOptions(pool);
			debug_logger->addAppender(debug_appender);

			debug_activated = true;
		}

		if(infoON)
		{
			info_logger = log4cxx::Logger::getLogger(logger_name_info.str());

			log4cxx::RollingFileAppenderPtr info_appender = new log4cxx::RollingFileAppender(layout, log_file_info.str(), false);

			info_appender->setMaximumFileSize(100*1024*1024);
			info_appender->setMaxBackupIndex(10);
			info_appender->activateOptions(pool);
			info_logger->addAppender(info_appender);

			info_activated = true;

		}

		if(warnON)
		{
			warn_logger = log4cxx::Logger::getLogger(logger_name_warn.str());

			log4cxx::RollingFileAppenderPtr warn_appender = new log4cxx::RollingFileAppender(layout, log_file_warn.str(), false);

			warn_appender->setMaximumFileSize(100*1024*1024);
			warn_appender->setMaxBackupIndex(10);
			warn_appender->activateOptions(pool);
			warn_logger->addAppender(warn_appender);

			warn_activated = true;
		}

		std::cout << std::endl;

	}

	void debug(const char* file, const char* function, int line, std::string msg)
	{
		if(ros_output)ROS_DEBUG_STREAM(msg);

		MyLocation = log4cxx::spi::LocationInfo(file,function,line);

		if(debug_activated == true)	debug_logger->debug(msg,MyLocation);

	}

	void info(const char* file, const char* function, int line, std::string msg)
	{
		if(ros_output)ROS_INFO_STREAM(msg);

		MyLocation = log4cxx::spi::LocationInfo(file,function,line);

		if(debug_activated == true)	debug_logger->info(msg, MyLocation);
		if(info_activated == true) info_logger->info(msg, MyLocation);

	}

	void warn(const char* file, const char* function, int line, std::string msg)
	{
		if(ros_output)ROS_WARN_STREAM(msg);

		MyLocation = log4cxx::spi::LocationInfo(file,function,line);

		if(debug_activated == true)	debug_logger->warn(msg, MyLocation);
		if(info_activated == true) info_logger->warn(msg, MyLocation);
		if(warn_activated == true) warn_logger->warn(msg, MyLocation);

	}

	void error(const char* file, const char* function, int line, std::string msg)
	{
		/*if (errno != 0)
		{
			msg.append(" | STDLIB error: '");
			msg.append(strerror(errno));
			msg.append("'");
			errno = 0;
		}*/

		if(ros_output)ROS_ERROR_STREAM(msg);

		MyLocation = log4cxx::spi::LocationInfo(file,function,line);

		if(debug_activated == true)	debug_logger->error(msg, MyLocation);
		if(info_activated == true) info_logger->error(msg, MyLocation);
		if(warn_activated == true) warn_logger->error(msg, MyLocation);

	}

	void fatal(const char* file, const char* function, int line, std::string msg)
	{
		if(ros_output)ROS_FATAL_STREAM(msg);

		MyLocation = log4cxx::spi::LocationInfo(file,function,line);

		if(debug_activated == true)	debug_logger->fatal(msg, MyLocation);
		if(info_activated == true) info_logger->fatal(msg, MyLocation);
		if(warn_activated == true) warn_logger->fatal(msg, MyLocation);

	}
}

