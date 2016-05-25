/***************************************************************************
 * Software License Agreement (BSD License)                                *
 *                                                                         *
 *  Copyright (c) 2012, Mikkel Hvilshoj, Christian Caroe, Casper Schou     *
 *	Department of Mechanical and Manufacturing Engineering             *
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

#ifndef LOGGERSYS_H_
#define LOGGERSYS_H_

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include "ros/ros.h"

#ifndef FFL
#define FFL __FILE__,__FUNCTION__,__LINE__
#endif

namespace skiros_common
{

	static std::stringstream log_msg;

	std::string findLogPath();

	void toggleOutput();

	void InitLogger(std::string package_name, bool debugON, bool infoON, bool warnON);

	void debug(const char* file, const char* function, int line, std::string msg);

	void info(const char* file, const char* function, int line, std::string msg);

	void warn(const char* file, const char* function, int line, std::string msg);

	void error(const char* file, const char* function, int line, std::string msg);

	void fatal(const char* file, const char* function, int line, std::string msg);


}

#define FDEBUG( msg )							\
{									\
	std::ostringstream os;                                          \
	os << msg;                                                      \
	skiros_common::debug(FFL, os.str());				\
}

#define FINFO( msg )							\
{									\
	std::ostringstream os;                                          \
	os << msg;                                                      \
	skiros_common::info(FFL, os.str());				\
}

#define FWARN( msg )                                               \
{                                                                   \
	std::ostringstream os;                                          \
	os << msg;                                                      \
	skiros_common::warn(FFL, os.str());											\
}

#define FERROR( msg )                                               \
{                                                                   \
	std::ostringstream os;                                          \
	os << msg;                                                      \
	skiros_common::error(FFL, os.str());											\
}

#define FFATAL( msg )                                               \
{                                                                   \
	std::ostringstream os;                                          \
	os << msg;                                                      \
	skiros_common::fatal(FFL, os.str());											\
}

#endif /* LOGGERSYS_H_ */

