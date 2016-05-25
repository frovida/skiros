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

#ifndef PARAMETER_MAP_H
#define PARAMETER_MAP_H

#include <map>
#include <boost/shared_array.hpp>
#include "skiros_common/param.h"
#include "skiros_common/logger_sys.h"

namespace skiros_common
{


/*! \brief Class which contain and manage a collection of generic parameters, protecting the access and specification.
 *
 *  Detailed description: A parameter can be added and, after that, cannot be erased or modified in it's description (it can just be specified)
 */
class ParamHandler
{
public:
		ParamHandler(void) {}

        ~ParamHandler()
        {
            params_.clear();
        }
		//------- Methods used inside skills -------------
        /*!
         * \brief Add a new parameter
         * \param p, the parameter
         * \return True on success
         */
        bool addParam(skiros_common::Param p);
        /*!
         * \brief Add un unspecified parameter to the list
         * \param key, a unique id for the parameter
         * \param description, a verbose description (optional)
         * \param param_type, the type of skiros parameter (more info in param.h).
         * \return true on success
         */
        template<class T>
        bool addParam(std::string key, std::string description="" , ParamSpecType param_type=skiros_common::offline)
        {
            skiros_common::Param p(key, description, typeid(T), param_type, 1);
            return addParam(p);
        }
        /*!
         * \brief Add a parameter with a default value
         * \param key, a unique id for the parameter
         * \param default_value, the default value. Note: char are automatically converted into std::string.
         * \param description, a verbose description (optional)
         * \param param_type, the type of skiros parameter (more info in param.h)
         * \return true on success
         */
        template<class T>
        bool addParamWithDefaultValue(std::string key, T default_value, std::string description="" , ParamSpecType param_type=skiros_common::online)
        {
            skiros_common::Param p(key, description, typeid(T), param_type, 1);
            p.setValue(default_value);
            if(!addParam(p))return false;
            return true;
        }
        /*template<class T>
        T * addParamWithDefaultValueG(std::string key, T default_value, std::string description="" , ParamSpecType param_type=skiros_common::online)
        {
            skiros_common::Param p(key, description, typeid(T), param_type, 1);
            p.setValue(default_value);
            if(!addParam(p))return 0;
            return getParamValuePtr<T>(key);
        }*/
        //! \brief Prototype, not to use
        template<class T>
        bool addConfigParam(std::string key, std::string description="")
        {
            return addParam<T>(key, description, skiros_common::config);
        }
        /*!
         * \brief get the whole parameter vector
         * \param key, the parameter id
         * \return The vector of values
         */
        template<class T> std::vector<T> getParamValues(std::string key)
        {
		  std::map<std::string, skiros_common::Param>::iterator it = params_.begin();
		  it=params_.find(key);
		  if (it == params_.end())
          {
			  FERROR("Element with key '" << key <<"' not found.");
              return std::vector<T>();
		  }
		  return it->second.getValues<T>();
		}
        /*!
         * \brief get the front parameter value
         * \param key, the parameter id
         * \return the front value in the vector
         */
		template<class T> T getParamValue(std::string key)
		{
		  std::vector<T> v = this->getParamValues<T>(key);
		  T to_ret;
		  if(v.size())to_ret = v[0];
		  return to_ret;
		}
        /*!
         * \brief get the parameter state
         * \param key, the parameter id
         * \return The parameter state (blank, initialized, specified)
         */
		skiros_common::ParameterState getParamState(std::string key)
		{
		  std::map<std::string, skiros_common::Param>::iterator it = params_.begin();
		  it=params_.find(key);
		  if (it == params_.end())
		  {
			  FERROR("Element with key '" << key <<"' not found.");
			  return skiros_common::blank;
		  }
		  return it->second.state();
		}
		//------- Methods used from manager -------------
        //! \brief Clear a parameter value
        void unspecify(std::string key);
        /*!
         * \brief specify value for a parameter
         * \param key, the parameter id
         * \param values, a vector of values to assign to the param. The vector must have the size as defined in the param and same datatype.
         * \return True on success
         */
		bool specify(std::string key, std::vector<skiros_common::any> values);
        /*!
         * \brief Specify value for a parameter exploiting the skiros_common::lexical_cast. Use carefully: it works only with a limited range of datatypes
         * \param key, the parameter id
         * \param values, a vector of values to assign to the param. The vector must have the size as defined in the param.
         * \return True on success
         */
        bool specify(std::string key, std::vector<std::string> values);
        /*!
         * \brief Specify value for a parameter of length 1
         * \param key, the parameter id
         * \param value, the value. The datatype must be the same as defined in the param description.
         * \return True on success
         */
        bool specify(std::string key, skiros_common::any value);
        /*!
         * \brief Specify value for a parameter of length 1. Templated version
         * \param key, the parameter id
         * \param value, the value. The datatype must be the same as defined in the param description.
         * \return True on success
         */
        template<class T> bool specify(std::string key, T value)
        {
            skiros_common::any a = value;
            specify(key, a);
        }
        /*!
         * \brief
         * \param key, the parameter id
         * \return Return true if param is specified
         */
        inline bool isSpecified(std::string key){return params_.at(key).isSpecified();}

        //! \brief Get a copy the parameters map
		std::map<std::string,skiros_common::Param> getParamMap();
		std::map<std::string,skiros_common::Param> getParamMap() const;

        /*!
         * \brief Get a partial copy the parameters map
         * \param type, the param type to be returned (all other are filtered out)
         * \return a copy of the param map filtered with the requested ParamSpecType
         */
        std::map<std::string,skiros_common::Param> getParamMapFiltered(ParamSpecType type);
protected:
        //! TODO. Now not used
        template<class T>
        int parseValues(T min_val, T max_val, T default_val);

        template<class T>
        /*!
         * \brief Only way to give access to the extern to the values of a param. This function is protected (allowed only when creating the param)
         * \return A vector of pointers to the internal param values
         */
        std::vector<T*> getParamValuesPtr(std::string key)
        {
            std::map<std::string, skiros_common::Param>::iterator it = params_.find(key);
            if (it == params_.end())
            {
                FERROR("Element with key '" << key <<"' not found.");
                return std::vector<T*>();
            }
            return it->second.getValuesPtr<T>();
        }
        template<class T>
        T * getParamValuePtr(std::string key)
        {
            std::map<std::string, skiros_common::Param>::iterator it = params_.find(key);
            if (it == params_.end())
            {
                FERROR("Element with key '" << key <<"' not found.");
                return 0;
            }
            return it->second.getValuePtr<T>();
        }

        skiros_common::ParamMap params_;
};
}

#endif // PARAMETER_MAP_H
