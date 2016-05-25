/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Francesco Rovida
 *	Robotics, Vision and Machine Intelligence Laboratory
 *  Aalborg University, Denmark
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
 *   * Neither the name of Aalborg Universitet nor the names of its
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
 *********************************************************************/

#include "skiros_common/param_handler.h"
#include "skiros_common/logger_sys.h"
 
namespace skiros_common
{
//------------------------------------------------------------------------------------------------------------

// Add new parameter:
bool ParamHandler::addParam(skiros_common::Param p)
{
	std::map<std::string, skiros_common::Param>::iterator it = params_.begin();
	it=params_.find(p.key());
	if (it != params_.end())
	{
		FERROR("Another element ( " << it->second.name() << " ) already exist with the same key " << p.key() <<".");
		return false;
	}
    params_.insert(it, std::pair<std::string, skiros_common::Param> (p.key(), p));
    return true;
}
//------------------------------------------------------------------------------------------------------------

void ParamHandler::unspecify(std::string key)
{
    std::map<std::string, skiros_common::Param>::iterator it = params_.find(key);
    if (it == params_.end())
        FERROR("Element with key '" << key <<"' not found.");
    it->second.unsetValues();
}

//------------------------------------------------------------------------------------------------------------

bool ParamHandler::specify(std::string key, skiros_common::any value)
{
    std::map<std::string, skiros_common::Param>::iterator it = params_.find(key);
    if (it == params_.end())
    {
        FERROR("Element with key '" << key <<"' not found.");
        return false;
    }
    if(it->second.setValue(value)) return true;
    else return false;
}

//------------------------------------------------------------------------------------------------------------

bool ParamHandler::specify(std::string key, std::vector<skiros_common::any> values)
{
    std::map<std::string, skiros_common::Param>::iterator it = params_.find(key);
	if (it == params_.end())
	{
		FERROR("Element with key '" << key <<"' not found.");
		return false;
	}
	if(it->second.setAllValues(values)) return true;
	else return false;
}


//------------------------------------------------------------------------------------------------------------

bool ParamHandler::specify(std::string key, std::vector<std::string> values)
{
	std::vector<skiros_common::any> values_any;
	for(int i=0;i<values.size();i++)
	{
		skiros_common::any any = values[i];
		values_any.push_back(any);
	}
	return this->specify(key, values_any);
}

//------------------------------------------------------------------------------------------------------------
// Get parameters map:
std::map<std::string,skiros_common::Param> ParamHandler::getParamMap()
{
	return params_;
}
// Get parameters map (const):
std::map<std::string,skiros_common::Param> ParamHandler::getParamMap() const
{
	return params_;
}


//------------------------------------------------------------------------------------------------------------

std::map<std::string,skiros_common::Param> ParamHandler::getParamMapFiltered(ParamSpecType type)
{
        std::map<std::string,skiros_common::Param> toRet;
        std::map<std::string, skiros_common::Param>::iterator it = params_.begin();
        for(it=params_.begin();it!=params_.end();it++)
        {
            if(it->second.specType()==type)toRet.insert(std::pair<std::string, skiros_common::Param>(it->first, it->second));
        }
        return toRet;
}

//------------------------------------------------------------------------------------------------------------

template<class T> int ParamHandler::parseValues(T min_val, T max_val, T default_val)
{
	FDEBUG("ParamHandler::parseValues");
	// Parse min/max values:
	if (max_val < min_val)
	{
		FERROR("Min larger than max. Should not happen!");
		return -1;
	}
	if (default_val < min_val)
	{
		FERROR("Default smaller than min. Should not happen!");
		return -1;
	}
	if (max_val < default_val)
	{
		std::cerr << "Default larger than max. Should not happen!" << std::endl;
		return -1;
	}

	return 0;
}

/*
 * TODO
 * template<class T> int ParamHandler::parseValues(vector<T> possible_values, T default_value)
{
	FDEBUG("ParamHandlerBase::parseValues");
	if (possible_values.size() == 0)
	{
		FERROR("Error: No possible values defined for skill parameter. Should not happen.");
		return -1;
	}

	bool default_value_found = false;
	for (int i = 0; i < (int)possible_values.size(); i++)
	{
		if (possible_values[i] == default_value)
		{
			default_value_found = true;
			break;
		}
	}
	if (!default_value_found)
	{
		FERROR("Error: Default value not defined as a possible value for skill parameter."
			 << " Should not happen. Changing to first possibility.");
		default_value = possible_values[0];
	}

	return 0;
}*/
}
