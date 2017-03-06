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

#ifndef PARAM_HPP
#define PARAM_HPP

//=================================
// Included dependencies
//=================================

#include <string>
#include <vector>
#include <typeinfo>       // operator typeid
#include <ros/ros.h>
#include "skiros_common/serializable_any.h"
#include "skiros_common/logger_sys.h"

namespace skiros_common
{
	//=================================
	// Forward declared dependencies
	//=================================
	class ParamLimit;

    /*!
     * \brief The parameter in SkiROS are found as (i) input for skills and (ii) properties for elements. This enum defines the different types of params.
     */
	enum ParamSpecType
	{
        //Skill input parameters
        online,             //A parameter that must be always specified
        offline,            //A parameter that can keep the default value, but cannot be left unspecified
        hardware,           //A parameter that specify the hardware to use
        templated,          //Not used for now. A templated parameter in the future will allow multiple input of the same parameter.
        optional,           //A parameter that can be left unspecified
        planning,           //A parameter for the planner
        config,             //A configuration parameter, loaded at boot
        //Properties(data) parameters
        symbolic,           //Descriptive data
        discrete            //Discrete data (used at a primitive level)
	};

    static const char * ParamSpecTypeStr[] =
    {
        "online",
        "offline",
        "hardware",
        "templated",
        "optional",
        "planning",
        "config",
        "symbolic",
        "discrete"
    };

	enum ParameterState
	{
		blank,
		initialized,
		specified
	};

	static const char * ParameterStateStr[] =
	{
			"Blank",
			"Initialized",
			"Specified"
	};

	//Class used to identify uninitialized params
	class undefined_class{};

	//This is a custom implementation of the omonim class of "std" library.
	//The original can't be embedded at the moment because need c++0x support
	//TODO CHECK because probably this works only when all the code is running on the same machine.(pointers problem)
	//NOTE: at the moment type_index can be compared to type_info only if it is on the LEFT side of an expression
	struct type_index {
	    type_index()
	    {
	    	target = &typeid(undefined_class);
	    }
	    type_index(const std::type_info& rhs)
	    {
	    	target = &rhs;
	    }
	    ~type_index()
	    {	    }

	    bool operator==(const std::type_info& rhs) const
	    {
	       return (rhs == *target);
	    }
	    bool operator==(const type_index& rhs) const
	    {
	       return (*rhs.target == *target);
	    }
	    bool operator!=(const std::type_info& rhs) const
	    {
		return (rhs != *target);
	    }
	    bool operator!=(const type_index& rhs) const
	    {
		return (*rhs.target != *target);
	    }
		/*size_t hash_code() const
		{
			return target->hash_code();
		}

		const char* name() const
		{
			return target->name();
		}*/

	    const std::type_info* target;
	    // Note that the use of a pointer here, rather than a reference,
	    // means that the default copy/move constructor and assignment
	    // operators will be provided and work as expected.
	};


	/*! \brief Class which define a parameter, which is a vector of eterogeneous values, limited in precise boundaries and referenced by a key
	 *
	 *  Detailed description: TODO
	 *
	 *  Depending on the type, there are standardized ways to teach the parameter. If the type is not know, the parameter must be teached from the skill
	 */

	class Param
	{
	public:
		Param(): state_(blank) {}
		Param(std::string key, std::string name): key_(key), name_(name), state_(blank) {}
        /*!
         * \brief Full initialization
         * \param key a unique id
         * \param name an optional description
         * \param value_type the datatype
         * \param type the parameter type in skiros (see ParamSpecType)
         * \param array_length the vector length
         */
		Param(std::string key, std::string name, const std::type_info &value_type, ParamSpecType type, int array_length=1 ):
			key_(key), name_(name)
		{
			this->init(value_type, type, array_length);
		}
		~Param() {}

		//Note: within this function is mandatory to give a vector type congruent with the type stored in the parameter.
		template<class T> std::vector<T>	getValues()
		{
			std::vector<T> v;
			if (state_ == blank)
			{
				FERROR("Trying to get value of uninitialized parameter: " << name_ << "! Aborted.");
				return v;
			}
			if (state_ == initialized)
			{
				FWARN("Trying to get value of unspecified parameter: " << name_ << "!");
				return v;
			}
			try
			{
			    for(int i=0;i<values_.size();i++)
			    {
				    v.push_back(skiros_common::any_cast<T>(values_[i]));
			    }
			}
			catch(const skiros_common::bad_any_cast &e)
			{
			    FERROR("Trying to get a value which doesn't correspond to the parameter type! Aborted.");
			    return v;
			}
			return v;
		}

        template<class T> T getValue(int index = 0)
		{
		  std::vector<T> v = this->getValues<T>();
          if(v.size()>index)return v[index];
		}

		template<class T> const T getValueConst()
		{
		  std::vector<T> v = this->getValues<T>();
		  if(v.size()>=1)return v[0];
		}

        template<class T>
        /*!
         * \brief Only way to modify directly, from the extern, the values of a param (NOTE: !unsafe!, use at your own risk)
         * \return A vector of pointers to the internal param values
         */
        std::vector<T*> getValuesPtr()
        {
            std::vector<T*> to_ret;
            for(int i=0;i<values_.size();i++)
            {
                T * temp = NULL;
                temp = skiros_common::any_cast<T>(&values_[i]);
                if(!temp)throw std::runtime_error("[Param::getValuesPtr] Fatal error.");
                to_ret.push_back(temp);
            }
            return to_ret;
        }
        template<class T>
        /*!
         * \brief Only way to modify directly, from the extern, the values of a param (NOTE: !unsafe!, use at your own risk)
         * \return A pointer to the internal param value (the first of the list)
         */
        T * getValuePtr()
        {
            return getValuesPtr<T>()[0];
        }

        skiros_common::any		getValue() const{return getValues()[0];}

        template<class T>
        /*!
         * \brief look for a value in the param vector
         * \param value the value to look for
         * \return the index of the first element matching the input value. -1 if no matching element is found
         */
        int find(T value)
        {
            std::vector<T> v = getValues<T>();
            for(int i=0;i<v.size();i++)
                if(v[i]==value)
                    return i;
            return -1;
        }

		std::vector<skiros_common::any>		getValues() const;

        bool					 getValues(std::vector<skiros_common::any>  &v);
        bool					 getValuesStr(std::vector<std::string> & v) const;
        std::vector<std::string> getValuesStr() const;
        /*!
         * \brief Print the the parameter values in a single output string
         * \return A string in the format [value_0, ..., value_n]
         */
        std::string             getValuesSingleStr() const;
        std::string             getValueStr() const;
        std::string             getValueStr(int index) const;

        std::string             key() const	      { return key_; }
        std::string             name() const      { return name_; }
        const std::type_info&	type() const      { return *value_type_.target; }
        bool                    isType(ParamSpecType type)    { return spec_type_==type; }
        std::string             typeName() const;
        ParameterState 			state() const     { return state_; }
        std::string             stateStr() const  { return std::string(ParameterStateStr[state_]); }
        ParamSpecType 			specType() const  { return spec_type_; }
        int                     size() const   { return values_.size();}
        bool                    isSpecified() const   { return this->state_ == skiros_common::specified; }

        //! \brief Print param data textually
        std::string printState() const;
        //! \brief Fix the Parameter value type and the vector length. The state goes into "initialized"
		void init(const std::type_info & value_type, ParamSpecType type, int array_length);
        //! \brief Reset the Param type to NULL and state to "blank"
		void reset();
        //! \brief Change the whole param description
		void reset(std::string key, std::string name, const std::type_info &value_type, ParamSpecType type, int array_length=1 );
        //! \brief Erase a value in the vector. The minimum vector size is 1
        void erase(int index);
        //! \brief Remove a value at the end of the vector. The minimum vector size is 1
        void pop_back();
        //! \brief Add a value at the end of the vector
        template<class T> bool push_back(T value)
        {
            values_.push_back(skiros_common::any());
            is_defined_.push_back(false);
            if(!this->setValue(value, values_.size()-1))
            {
                //revert on failure
                this->pop_back();
                return false;
            }
            else return true;
        }

        inline void                     unsetValue(){unsetValues();}
        //! \brief Unset the parameter values
        void                            unsetValues();

        //! \brief Check If ParamType is specified and equal to value type. Then set the type
        template<class T> bool			setValue(T value, int index = 0)
        {
          skiros_common::any va = value;
          return this->setValue(va, index);
		}
        bool                            setValue(skiros_common::any value, int index = 0);

		template<class T> bool			setAllValues(std::vector<T> values)
		{
		    std::vector<skiros_common::any> va;
		    skiros_common::any a;
		    for(int i=0; i<values.size();i++)
		    {
			a = values[i];
			va.push_back(a);
		    }
		    return this->setAllValues(va);
		}
        bool                            setAllValues(std::vector<skiros_common::any> values);
        /*!
         * \brief Set the param values
         * \param values, an input string in the format [value_0, ..., value_n]
         * \return True if the value are succesfully set, false otherwise
         */
        bool                            setAllValuesFromSingleStr(std::string values);

		//----------------- LIMITS SPECIFICATION ------------------------------------
        //! \brief Set the default value of the parameter NOTFINISHED
		bool setDefault(skiros_common::any default_value);
        //! \brief Set the limit for the parameter specification NOTFINISHED
		bool setLimit(std::vector<skiros_common::any>  values, int default_value, double step_size = 0);
        //! \brief Get the limit of the parameter NOTFINISHED
		ParamLimit getLimit();
        //! \brief Reset the limit NOTFINISHED
		void resetLimit();

		//--------------------- OPERATORS --------------------------------
        //! \brief compare if two params have the same type and vector length
		bool operator==(const Param& rhs) const;
        //! \brief compare if two params have different type and vector length
		bool operator!=(const Param& rhs) const;

		//TODO: all the operators (= < > etc) (valid only when vector length = 1). Important the = for deep copy!!
		//TODO: define a = that copy only the values_
	protected:
        typedef std::vector<skiros_common::any> ValueType;
        typedef std::vector<bool> DefinedType;
        ValueType               values_;
        DefinedType             is_defined_;
		std::string 			key_;				// For task file, short
		std::string 			name_;				// For UI, more descriptive
		ParameterState 			state_;
        type_index              value_type_;
		ParamSpecType 			spec_type_;
        ros::Time               last_update_;
		ParamLimit *			limit_;
		//TODO Unit of measure?
		//TODO Tolerances?
		void checkSpecified()
		{
			bool isSpecified = true;
			//Check if all values of vector have been specified to set the parameter state to specified
			for (int i=0; i<is_defined_.size(); i++)
			{
			    if(!is_defined_[i]) isSpecified=false;
			}
			if(isSpecified)state_ = specified;
		}

		template<class T> void convert(std::vector<skiros_common::any> &values);
        std::string toString(skiros_common::any value) const;
	};

    /*!
     * \brief This is associated to a parameter and defines constraints over its values.
     */
	class ParamLimit
	{
	public:
		ParamLimit(skiros_common::any default_value)
		{
		  possible_values_.push_back(default_value);
		  default_value_ = 0;
		}

		ParamLimit(std::vector<skiros_common::any>  values, int default_value, double step_size = 0)
		{
		  possible_values_ = values;
		  if(default_value >= 0 && default_value < possible_values_.size()) default_value_ = default_value;
		  else default_value_ = 0;
		  //Is possible to define a step only if are present no more than min, max and default values.
		  if(possible_values_.size() < 3) step_size_ = step_size;
		  else step_size_ = 0;
		}

		skiros_common::any getDefaultValue(void) { return possible_values_[default_value_]; }
		skiros_common::any getMin(void) { return possible_values_[0]; }
		skiros_common::any getMax(void) { return possible_values_[possible_values_.size()-1]; }
		double getStepSize(void) { return step_size_; }
		std::vector<skiros_common::any> getPossibleValues() { return possible_values_;}
		//std::vector<std::string> getValuesTxt();
	private:
		int default_value_;					// Index of the default value
		double step_size_;					// Set a discrete step (double choosed because is the most generic numeric type)
		//Maybe TODO: implement a function to define a variable step?
		//When vectorSize == 1, we have only the default specifcation, when == 2 or 3, we have min, max and default, if >3 we have specific values.
		std::vector<skiros_common::any> possible_values_;
	};

	typedef std::map<std::string, skiros_common::Param> ParamMap;
    typedef std::pair<std::string, skiros_common::Param> ParamPair;
}


namespace ros
{
namespace serialization
{
  template<>
  struct Serializer<std::vector<skiros_common::any> >
  {//TODO: serialize the std::vector<bool>
    template<typename Stream>
    inline static void write(Stream& stream, const std::vector<skiros_common::any>& v)
    {
      stream.next(v.size());
      for(int i=0; i<v.size();i++)
      {
          stream.next(v[i]);
      }
    }
    template<typename Stream>
    inline static void read(Stream& stream, std::vector<skiros_common::any>& v)
    {
      int size;
      stream.next(size);
      skiros_common::any my_amy;
      for(int i=0; i<size;i++)
      {
          stream.next(my_amy);
          v.push_back(my_amy);
      }
    }

    inline static uint32_t serializedLength(const std::vector<skiros_common::any>& v)
    {
      uint32_t size = 0;
      size += serializationLength(v.size());
      for(int i=0; i<v.size();i++)
      {
          size += serializationLength(v[i]);
      }
      return size;
    }
  };

  template<>
  struct Serializer<skiros_common::Param>
  {//TODO: serialize the std::vector<bool>
    template<typename Stream>
    inline static void write(Stream& stream, const skiros_common::Param& p)
    {
      stream.next(p.key());
      stream.next(p.name());
      stream.next((int)p.state());
      stream.next((int)p.specType());
      stream.next(p.size());
      if(p.state() == skiros_common::specified)
      {
          std::vector<skiros_common::any> v = p.getValues();
          for(int i=0; i<v.size();i++)
          {
              stream.next(v[i]);
          }
      }
      else
      {
          stream.next(skiros_common::any::getHashFromType(p.type()));
      }
    }

    template<typename Stream>
    inline static void read(Stream& stream, skiros_common::Param& p)
    {
      std::string key;
      std::string name;
      int state;
      int specType;
      int size;
      uint64_t type_hash;
      stream.next(key);
      stream.next(name);
      stream.next(state);
      stream.next(specType);
      stream.next(size);
      if((skiros_common::ParameterState) state == skiros_common::specified)
      {
          std::vector<skiros_common::any> v;
          skiros_common::any my_amy;
          for(int i=0; i<size;i++)
          {
              stream.next(my_amy);
              v.push_back(my_amy);
          }
          p.reset(key, name, v[0].type(), (skiros_common::ParamSpecType) specType, size);
          p.setAllValues(v);
      }
      else
      {
          stream.next(type_hash);
          p.reset(key, name, skiros_common::any::getTypeFromHash(type_hash), (skiros_common::ParamSpecType) specType, size);
      }
    }

    inline static uint32_t serializedLength(const skiros_common::Param& p)
    {
      uint32_t size = 0;
      size += serializationLength(p.key());
      size += serializationLength(p.name());
      size += serializationLength((int)p.state());
      size += serializationLength((int)p.specType());
      size += serializationLength(p.size());
      if(p.state() == skiros_common::specified)
      {
          std::vector<skiros_common::any> v = p.getValues();
          //if(v.size()==0)ROS_INFO("E STI CAZZI");
          //else ROS_INFO("E STI RAZZI");
          for(int i=0; i<v.size();i++)
          {
              size += serializationLength(v[i]);
          }
      }
      else
      {
          uint64_t temp;
          size += serializationLength(temp);
      }
      return size;
    }
  };


  template<>
  struct Serializer<skiros_common::ParamMap>
  {
    template<typename Stream>
    inline static void write(Stream& stream, const skiros_common::ParamMap& t)
    {
      skiros_common::ParamMap::const_iterator it = t.begin();
      stream.next((uint32_t)t.size());
      for(it=t.begin();it!=t.end();it++)
      {
          stream.next(it->second);
      }
    }

    template<typename Stream>
    inline static void read(Stream& stream, skiros_common::ParamMap& t)
    {
      uint32_t num_of_param;
      stream.next(num_of_param);
      skiros_common::Param p;
      skiros_common::ParamMap::iterator it;
      t.clear();
      if(num_of_param)
      {
        for(int i=0; i<num_of_param;i++)
        {
            stream.next(p);
            it = t.end();
            t.insert(it, std::pair<std::string, skiros_common::Param> (p.key(), p));
        }
      }
    }

    inline static uint32_t serializedLength(const skiros_common::ParamMap& t)
    {
      skiros_common::ParamMap::const_iterator it = t.begin();
      uint32_t size = 0;
      uint32_t num_of_param;
      size += serializationLength(num_of_param);
      for(it=t.begin();it!=t.end();it++)
      {
          size += serializationLength(it->second);
      }
      return size;
    }
  };



}
}

#endif // PARAM_HPP
