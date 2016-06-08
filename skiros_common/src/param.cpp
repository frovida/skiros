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

#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include "skiros_common/param.h"
//An implementation of Param using skiros_common/any

//TODO:
//General: throw exceptions.
//setValue: take counts of the parameters limits
/***********************************/
/************** Param **************/
/***********************************/



namespace skiros_common
{
#ifdef __GNUG__

    #include <cxxabi.h>
    #include <cstdlib>
    #include <memory>

    std::string demangleType(const std::type_info & type )
    {
        int status ;
        std::unique_ptr< char[], decltype(&std::free) > buffer(
            __cxxabiv1::__cxa_demangle( type.name(), nullptr, 0, &status ), &std::free ) ;
        return status==0 ? buffer.get() : "__cxa_demangle error" ;
    }

#else // !defined __GNUG__

    std::string demangleType(const std::type_info & type) { return type.name(); }

#endif //__GNUG__

/*bool Param::setValue(skiros_common::any value, int position)
{
	FDEBUG("Param::setValue (" << this->getKey() << ") with type" << value.type().name());
	if(state_ != blank )
	{
		if(value_type_ == value.type() && position < values_.size())
		{
			values_.at(position) = value;
			is_defined_.at(position) = true;
			this->checkSpecified();
			FDEBUG("setValue done.");
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}*/
/*
template<class T> bool Param::setAllValues(std::vector<T> values)
{
	if(value_type_ != typeid(T))
	{
		FERROR("Trying to set a value which doesn't correspond to the parameter type! Aborted.");
		return false;
	}
	if(values.size() != values_.size())
	{
		std::ostringstream ss;
		ss << "Input vector length doesn't coincide with the parameter: " << values_.size();
		FERROR(ss.str());
		return false;
	}
	T val;
	for (int i = 0;i<values_.size();i++)
	{
		val = values.at(i);
		values_.at(i) = val;
	}
	for (int i=0; i<is_defined_.size(); i++) is_defined_.at(i) = true;
	state_ = specified;
	return true;
}*/
//-------------------------------------------------------------

std::string Param::getValuesSingleStr() const
{
    std::stringstream to_ret;
    to_ret << "[";
    for(int i=0;i<values_.size();i++)
    {
        to_ret << getValueStr(i);
        if(i!=values_.size()-1)
            to_ret << ",";
    }
    to_ret << "]";
    return to_ret.str();
}

//-------------------------------------------------------------

//std::stringstream Param::printState()
std::string Param::printState() const
{
  std::stringstream ss;
  std::vector<std::string> v;
  ss << key_;
  this->getValuesStr(v);
  ss << " [ ";
  for(int i=0; i<v.size();i++)
  {
          ss << v[i];
          if(v.size()-1==i) ss << " ";
          else ss << ", ";
  }
  ss << "]" << " " << demangleType(type()) << ", " << ParamSpecTypeStr[specType()]; // << " State: " << ParameterStateStr[state_];
  //ROS_INFO_STREAM(ss.str());
  return ss.str();
}

//-------------------------------------------------------------

std::string Param::typeName(void) const
{
    return demangleType(type());
}

//-------------------------------------------------------------

std::vector<skiros_common::any>  Param::getValues() const
{
      if (state_ == specified)
      {
          return values_;
      }
      else
      {
          std::vector<skiros_common::any> v;
          return v;
      }
}

//-------------------------------------------------------------

bool Param::getValues(std::vector<skiros_common::any> &v)
{
	if (state_ == blank)
	{
		FERROR("Trying to get value of uninitialized parameter: " << name_ << "! Aborted.");
		return false;
	}
	if (state_ == initialized)
	{
		FWARN("Trying to get values of unspecified parameter: " << name_ << ".");
		//return false;
	}
	v.clear();
	v = values_;
	return true;
}

//-------------------------------------------------------------

void Param::unsetValues()
{
    init(type(), specType(), size());
}

//-------------------------------------------------------------

bool Param::getValuesStr(std::vector<std::string> &v) const
{
    v.clear();
    for(int i=0;i<values_.size();i++)
    {
        if(is_defined_[i]) v.push_back(this->toString(values_[i]));
        else v.push_back("N/A");
    }
	return true;
}
//-------------------------------------------------------------

std::vector<std::string> Param::getValuesStr() const
{
    std::vector<std::string> v;
    getValuesStr(v);
    return v;
}

//-------------------------------------------------------------

std::string Param::getValueStr() const
{
    std::stringstream ss;
    for(int i=0;i<values_.size();i++)
    {
        if(is_defined_[i]) ss << this->toString(values_[i]);
        else ss << "N/A";
        if(i+1<values_.size()) ss << ",";
    }
    return ss.str();
}
//-------------------------------------------------------------

std::string Param::getValueStr(int index) const
{
    if(index>=0 && index<values_.size())
    {
        if(is_defined_[index]) return this->toString(values_[index]);
        else return "N/A";
    }
    else throw std::invalid_argument("[Param::getValueStr] Out of boundaries.");
}

//-------------------------------------------------------------

void Param::init(const std::type_info &value_type, ParamSpecType type, int array_length)
{
	value_type_ = value_type;
    //Convert char const to string (char const often give problems)
    if(value_type_==typeid(char const*)) value_type_=typeid(std::string);
	values_.clear();
	is_defined_.clear();
	//values_.get_allocator().allocate(array_length);
	for (int i=0; i<array_length; i++)
	{
		values_.push_back(skiros_common::any());
		is_defined_.push_back(false);
	}
	spec_type_ = type;
	state_ = initialized;
	last_update_ = ros::Time::now();
}

//-------------------------------------------------------------

void Param::erase(int index)
{
    ValueType::iterator it = values_.begin();
    DefinedType::iterator it2 = is_defined_.begin();
    for(int i=0;i<index;i++)
    {
        it++;
        it2++;
    }
    if(values_.size()>1)
    {
        values_.erase(it);
        is_defined_.erase(it2);
        this->checkSpecified();
    }

}

//-------------------------------------------------------------

void Param::pop_back()
{
    if(values_.size()>1)
    {
        values_.pop_back();
        is_defined_.pop_back();
        this->checkSpecified();
    }
}

//-------------------------------------------------------------

void Param::reset()
{
	values_.clear();
	is_defined_.clear();
	state_ = blank;
	value_type_ = typeid(undefined_class);
}

//-------------------------------------------------------------

void Param::reset(std::string key, std::string name, const std::type_info &value_type, ParamSpecType type, int array_length )
{
	values_.clear();
	is_defined_.clear();
	state_ = blank;
	value_type_ = typeid(undefined_class);
	key_ = key;
	name_ = name;
	this->init(value_type, type, array_length );
}

//-------------------------------------------------------------

//Tries to convert the input vector into the parameter type, if it fails return false
bool Param::setAllValues(std::vector<skiros_common::any> values)
{
	//Check on lenght
	if(values.size() != values_.size())
	{
		std::ostringstream ss;
		ss << "Input vector length doesn't coincide with the parameter: " << values_.size();
		FERROR(ss.str());
		return false;
	}
	//Check on vector type consistancy
	for(int i=1; i<values.size();i++)
	{
		if(values[i].type() != values.front().type())
		{
			FERROR("All elements of the input vector must be of the same type." );
			return false;
		}
	}
	//If input typeid doesn't coincide with the parameter one, try automatic convert
	if(value_type_ != values.front().type())
	{
		try
		{
			//TODO: extend the handled input type
			if(value_type_ == typeid(int))
			{
					this->convert<int>(values);
			}
			else if(value_type_ == typeid(bool))
			{
					this->convert<bool>(values);
			}
			else if(value_type_ == typeid(double))
			{
				this->convert<double>(values);
			}
			else if(value_type_ == typeid(std::string))
			{
				//Note: this function leaves the parameters set to "N/A" like they are. N/A is the special caracter for undefined params
				this->convert<std::string>(values);
			}
			else
			{
                FERROR(this->key() << ": Unhandled input type - " << values.front().type().name());
				return false;
			}
		}
		catch ( const boost::bad_any_cast &e )	{FERROR("Input values cannot be converted into parameter type. ");return false;}
		catch ( const boost::bad_lexical_cast &e )	{FERROR("Input values cannot be converted into parameter type. ");return false;}
	}
	//Assign the values. (Only the defined ones)
	for(int i=0; i<values.size();i++)
	{
		bool assign = true;
		if(values[i].type() == typeid(std::string))
		{
			std::string str = skiros_common::any_cast<std::string>((values)[i]);
			if(str == "N/A") assign = false;
		}
		if(assign)
		{
			is_defined_[i] = true;
			values_[i] = values[i];
		}
	}
	//Check to set the state to specified
	this->checkSpecified();
	last_update_ = ros::Time::now();
	return true;
}

//-------------------------------------------------------------

std::vector<std::string> & split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

bool Param::setAllValuesFromSingleStr(std::string values)
{
    //Remove [ and ]
    size_t start = values.find('[');
    size_t end = values.find(']');
    if(start==std::string::npos || end==std::string::npos)
        return false;
    values = values.substr(start+1, end-1);
    //Split
    std::vector<std::string> v = split(values, ',');
    return this->setAllValues(v);
}

//------------------------------------------------------------- TODO lexical_cast doesn't work with bool TODO: any cast doesn't WORK!

bool Param::setValue(skiros_common::any value, int index)
{
    //If input typeid doesn't coincide with the parameter one, try automatic convert
    if(value_type_ != value.type())
    {
        std::vector<skiros_common::any> values;
        values.push_back(value);
        try
        {
            //TODO: extend the handled input type
            if(value_type_ == typeid(int))
            {
                    this->convert<int>(values);
            }
            else if(value_type_ == typeid(bool))
            {
                    this->convert<bool>(values);
            }
            else if(value_type_ == typeid(double))
            {
                this->convert<double>(values);
            }
            else if(value_type_ == typeid(std::string))
            {
                //Note: this function leaves the parameters set to "N/A" like they are. N/A is the special caracter for undefined params
                this->convert<std::string>(values);
            }
            else
            {
                FERROR(this->key() << ": Unhandled input type - " << values.front().type().name());
                return false;
            }
        }
        catch ( const boost::bad_any_cast &e )	{FERROR("Input values cannot be converted into parameter type. ");return false;}
        catch ( const boost::bad_lexical_cast &e )	{FERROR("Input values cannot be converted into parameter type. ");return false;}
        value = values[0];
    }
    //Assign the value
    bool assign = true;
    if(value.type() == typeid(std::string))
    {
        std::string str = skiros_common::any_cast<std::string>(value);
        if(str == "N/A") assign = false;
    }
    if(assign)
    {
        is_defined_[index] = true;
        values_[index] = value;
    }
    //Check to set the state to specified
    this->checkSpecified();
    last_update_ = ros::Time::now();
    return true;
}

//------------------------------------------------------------- TODO lexical_cast doesn't work with bool TODO: any cast doesn't WORK!

template<class T> void Param::convert(std::vector<skiros_common::any> &values)
{
	FDEBUG("Values doesn't correspond to the parameter type. Convert:"<< values.front().type().name() << " to: " << value_type_.target->name());
	try
	{
	  if(value_type_ == typeid(std::string) &&  values.front().type() == typeid(char const*))
	  {
	     for(int i=0; i<values.size();i++)
	     {
		     values[i] = std::string(skiros_common::any_cast<char const*>(values[i]));
	     }
	  }
      else if(values.front().type() == typeid(std::string) || values.front().type() == typeid(char const*))
	  {
          //The input value is a string, use lexical_cast
		  for(int i=0; i<values.size();i++)
		  {
			  std::string str = skiros_common::any_cast<std::string>(values[i]);
              if(str != "N/A")
              {
                  if(typeid(T)==typeid(bool) && str=="true") values[i] = true;
                  else if(typeid(T)==typeid(bool) && str=="false") values[i] = false;
                  else values[i] = boost::lexical_cast<T>(str.c_str());
              }
		  }
	  }
	  else
	  {
		  //Else must use any_cast (doesn't work!)
		  for(int i=0; i<values.size();i++)
		  {
			   values[i] = skiros_common::any_cast<T>(values[i]);
		  }
	  }
	}
	catch ( const boost::bad_any_cast &e )	{throw e;}
	catch ( const boost::bad_lexical_cast &e )	{throw e;}
}

//------------------------------------------------------------- CHECKED: double, int, bool, std::string, char TODO: other

std::string Param::toString(skiros_common::any  value) const
{
    std::stringstream ss;
    ss << value;
	return ss.str();
}

//-------------------------------------------------------------
//Set the default value of the parameter
bool Param::setDefault(skiros_common::any default_value)
{
  if(limit_ != NULL) return false;
  else
  {
      limit_ = new ParamLimit(default_value);
      return true;
  }
}

//-------------------------------------------------------------
//Set the limit for the parameter specification
bool Param::setLimit(std::vector<skiros_common::any>  values, int default_value, double step_size)
{
  if(limit_ != NULL) return false;
  else
  {
      limit_ = new ParamLimit(values, default_value, step_size);
      return true;
  }
}

//-------------------------------------------------------------

//Get the limit of the parameter
ParamLimit Param::getLimit()
{
  return *limit_;
}

//-------------------------------------------------------------

//Reset the limit
void Param::resetLimit()
{
  delete limit_;
}

//------------------------------------------------------------- TODO (URGENT) check that also the value is the same!!!
bool Param::operator==(const Param& rhs) const
{
  if(value_type_ != rhs.type() || this->size() != rhs.size() || this->state() != specified || rhs.state() != specified) return false;
  return true;
/*  if(value_type_ == typeid(bool))
  {
      std::vector<bool> v;
      rhs.getValues<bool>(v);
      for(int i=0; i<this->size();i++)
      {
          if(skiros_common::any_cast<bool>(values_[i])!=  skiros_common::any_cast<bool>(v[i]))return false;
      }
  }
  else if(value_type_ == typeid(double))
  {
      std::vector<double> v;
      rhs.getValues<double>(v);
      for(int i=0; i<this->arrayLenght();i++)
      {
          if(skiros_common::any_cast<double>(values_[i])!=  skiros_common::any_cast<double>(v[i]))return false;
      }
  }
  else if(value_type_ == typeid(std::string))
  {
      std::vector<std::string> v;
      rhs.getValues<std::string>(v);
      for(int i=0; i<this->arrayLenght();i++)
      {
          if(skiros_common::any_cast<std::string>(values_[i])!=  skiros_common::any_cast<std::string>(v[i]))return false;
      }
  }
  else if(value_type_ == typeid(Element))
  {
      std::vector<Element> v;
      rhs.getValues<Element>(v);
      for(int i=0; i<this->arrayLenght();i++)
      {
          if(skiros_common::any_cast<Element>(values_[i])!=  skiros_common::any_cast<Element>(v[i]))return false;
      }
  }
  else return true;*/
}

//------------------------------------------------------------- TODO (URGENT) check that also the value is the same!!!

bool Param::operator!=(const Param& rhs) const
{
  if(this->type() != rhs.type() || this->size() != rhs.size()) return true;
  if(this->state() != specified || rhs.state() != specified) return false;
  //If both are specified check if the value is the same
  //This can be difficult in case of complex types TODO
  return false;
}
}
