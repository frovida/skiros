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

// Modified from boost 1.37 boost::any
// Extended to handle ros serialization/deserialization functions
// See http://www.boost.org/libs/any for Documentation.

#ifndef SERIALIZABLE_ANY
#define SERIALIZABLE_ANY

#include <stdexcept>
#include <boost/any.hpp>
#include <ros/ros.h>
#include <ostream>

namespace skiros_common
{

  class any;
  class placeholder
  {
  public: // structors

      virtual ~placeholder()
      {
      }

  public: // queries

      virtual const std::type_info & type() const = 0;

      virtual placeholder * clone() const = 0;

  public: // serialization
    virtual const std::type_info & get_type() = 0;
    //static placeholder* baseRosSerialOut(ros::serialization::OStream& arc);
    //template <typename Stream>
    static placeholder *baseRosSerialIn(ros::serialization::IStream& stream);
    //template <typename Stream>
    virtual void baseRosSerialOut(ros::serialization::OStream& stream) = 0;
    //virtual void rosSerialIn(ros::serialization::IStream& stream) = 0;
    virtual uint32_t rosSerialLength() const = 0;
  public: //visualization
    virtual void print(std::ostream &o, const any &a) const =0;
  };

  typedef placeholder* (*any_registration_deserializer_type)(ros::serialization::IStream& stream, bool true_stream);
  //Map between type code and placeholder pointer
  typedef std::map<uint64_t, any_registration_deserializer_type> any_registration_map_type;

  //Singleton
  any_registration_map_type& getRegistrationMap();

  class bad_serialization : public std::runtime_error
  {
    public:
    explicit bad_serialization(const std::string& what_arg):
    std::runtime_error("skiros_common::bad_serialization: "+what_arg)
    {
    }
  };

  class any
  {
  public: // structors

      any()
        : content(0){ }

      template<typename ValueType>
      any(const ValueType & value)
        : content(new any::holder<ValueType>(value))
      {
        // force instantiation of the registration template (without running the constructor)
        any::holder<ValueType>::registration.get_deserializer_id();
      }

      any(const any & other)
        : content(other.content ? other.content->clone() : 0){}

      ~any()
      {
          delete content;
      }

  public: // modifiers

      any & swap(any & rhs)
      {
          std::swap(content, rhs.content);
          return *this;
      }

      template<typename ValueType>
      any & operator=(const ValueType & rhs)
      {
          any::holder<ValueType>::registration.get_deserializer_id();
          any(rhs).swap(*this);
          return *this;
      }

      template<typename ValueType>
      ValueType * operator=(any & rhs);

      any & operator=(any rhs)
      {
          rhs.swap(*this);
          return *this;
      }

  public: // queries

      bool empty() const
      {
          return !content;
      }

      const std::type_info & type() const
      {
          return content ? content->type() : typeid(void);
      }

  public: // serialization


      static const std::type_info & getTypeFromHash(uint64_t hash)
      {
        any_registration_map_type map = getRegistrationMap();
        any_registration_map_type::iterator i = map.find(hash);
        assert(i != map.end());
        uint8_t * fake_data = new uint8_t[0];
        ros::serialization::IStream stream(fake_data, 0);
        return map[hash](stream,false)->get_type();
      }

      static const uint64_t getHashFromType(const std::type_info & type)
      {
        return type_hashing(type);
      }

      template<typename Stream>
      void rosSerialOut(Stream& stream) const
      {
        //const std::type_info tt = this->type();
       // any_cast<tt>(this);
        if(typeid(stream) == typeid(ros::serialization::OStream))
        {
            bool isEmpty = empty();
            stream.next(isEmpty);
            content->baseRosSerialOut((ros::serialization::OStream&)stream);
        }
        else
        {
          std::stringstream str;
          str << "Not possible to serialize skiros_common::any with stream type " << typeid(stream).name();
          throw bad_serialization(str.str());
        }
      }
      template<typename Stream>
      void rosSerialIn(Stream& stream)
      {
        if(typeid(stream) == typeid(ros::serialization::IStream))
        {
           if(content != NULL) delete content;
           bool isempty;
           stream.next(isempty);
           if (isempty == false)
           {
             content = placeholder::baseRosSerialIn(stream);
           }
        }
        else
        {
          std::stringstream str;
          str << "Not possible to deserialize skiros_common::any with stream type " << typeid(stream).name();
          throw bad_serialization(str.str());
        }
      }

      uint32_t rosSerialLength() const
      {
        bool isEmpty = empty();
        uint32_t temp = ros::serialization::serializationLength(isEmpty);
        if(!isEmpty) temp += content->rosSerialLength();
        return temp;
      }

  public: // types (public so any_cast can be non-friend)

      template <typename T>
      class any_registration
      {
      public:
          any_registration()
          {
               getRegistrationMap()[any_registration<T>::get_deserializer_id()] = any_registration<T>::deserialize;
             //std::cout << "registered " << typeid(T).name() << " to " << get_deserializer_id() << std::endl;
          }

         static bool inited; // whether localid has been created
         static uint64_t localid;  // cached id of this type. Avoids rehashing everything I serialize

         static uint64_t get_deserializer_id()
         {
           if (inited == false) compute_deserializer_id();
           return localid;
         }

         static void compute_deserializer_id()
         {
           inited = true;
           localid = any::type_hashing(typeid(T));
         }

         static placeholder* deserialize(ros::serialization::IStream &arc, bool true_stream)
         {
           any::holder<T> *newholder = new any::holder<T>(arc, true_stream);
           return newholder;
         }
      };

      template<typename ValueType>
      class holder : public placeholder
      {
      public: // structors

          holder(const ValueType & value)
            : held(value)
          {
          }
          //template<typename Stream>
          holder(ros::serialization::IStream& stream, bool true_stream)
          {
            if(true_stream) ros::serialization::Serializer<ValueType>::read(stream, held);
          }

      public: // queries

          virtual const std::type_info & type() const
          {
              return typeid(ValueType);
          }

          virtual placeholder * clone() const
          {
              return new holder(held);
          }

      public: // serialization

        static any_registration<ValueType> registration;

        virtual const std::type_info & get_type()
        {
          return typeid(ValueType);
        }

        //template<typename Stream>
        virtual void baseRosSerialOut(ros::serialization::OStream& stream)
        {
          stream.next(registration.get_deserializer_id());
          ros::serialization::Serializer<ValueType>::write(stream, held);
        }

        uint32_t rosSerialLength() const
        {
            return ros::serialization::serializationLength(registration.get_deserializer_id()) + ros::serialization::Serializer<ValueType>::serializedLength(held);
        }

        void print(std::ostream &o, const any &a) const{ o << any_cast<ValueType>(const_cast<any &>(a));}
      public: // representation

          ValueType held;

      private: // intentionally left unimplemented
          holder & operator=(const holder &);
      };

  private: // representation

      static uint64_t  type_hashing(const std::type_info & type)
      {
          // FNV hash function
          const char *p = type.name();
          uint64_t h = 2166136261u;
          while((*p) != 0) {
            h = ( h * 16777619 ) ^ ((unsigned char)(*p));
            ++p;
          }
          return h;
      }

      template<typename ValueType>
      friend ValueType * any_cast(any *);

      template<typename ValueType>
      friend ValueType * unsafe_any_cast(any *);

      friend std::ostream &operator<<(std::ostream& o, const any & a) {
        if(a.content)
            a.content->print(o, a);
        return o;
      }

      placeholder * content;
  };

  class bad_any_cast : public std::bad_cast
  {
  public:
      virtual const char * what() const throw()
      {
          return "skiros_common::bad_any_cast: "
                 "failed conversion using skiros_common::any_cast";
      }
  };

  template<typename ValueType>
  ValueType * any_cast(any * operand)
  {
      return operand &&
#ifdef BOOST_AUX_ANY_TYPE_ID_NAME
          std::strcmp(operand->type().name(), typeid(ValueType).name()) == 0
#else
          operand->type() == typeid(ValueType)
#endif
          ? &static_cast<any::holder<ValueType> *>(operand->content)->held
          : 0;
  }

  template<typename ValueType>
  inline const ValueType * any_cast(const any * operand)
  {
      return any_cast<ValueType>(const_cast<any *>(operand));
  }

  template<typename ValueType>
  ValueType any_cast(any & operand)
  {
      ValueType * result = any_cast<ValueType>(&operand);
      if(!result)boost::throw_exception(bad_any_cast());
      return *result;
  }

  template<typename ValueType>
  inline ValueType any_cast(const any & operand)
  {
      return any_cast<const ValueType &>(const_cast<any &>(operand));
  }

  template<typename ValueType>
  ValueType * any::operator=(any & rhs)
  {
      return any_cast<ValueType>(const_cast<any &>(rhs));
  }
  // Note: The "unsafe" versions of any_cast are not part of the
  // public interface and may be removed at any time. They are
  // required where we know what type is stored in the any and can't
  // use typeid() comparison, e.g., when our types may travel across
  // different shared libraries.
  template<typename ValueType>
  inline ValueType * unsafe_any_cast(any * operand)
  {
      return &static_cast<any::holder<ValueType> *>(operand->content)->held;
  }

  template<typename ValueType>
  inline const ValueType * unsafe_any_cast(const any * operand)
  {
      return unsafe_any_cast<ValueType>(const_cast<any *>(operand));
  }

  template<typename T> any::any_registration<T> any::holder<T>::registration;
  template<typename T> bool any::any_registration<T>::inited = false;
  template<typename T> uint64_t any::any_registration<T>::localid;

} //namespace skiros_common


namespace ros
{
namespace serialization
{
  template<>
  struct Serializer<skiros_common::any>
  {
    template<typename Stream>
    inline static void write(Stream& stream, const skiros_common::any& t)
    {
      t.rosSerialOut(stream);
    }

    template<typename Stream>
    inline static void read(Stream& stream, skiros_common::any& t)
    {
      t.rosSerialIn(stream);
    }

    inline static uint32_t serializedLength(const skiros_common::any& t)
    {
      return t.rosSerialLength();
    }
  };
  template<>
  struct Serializer<const char*>
  {
    template<typename Stream>
    inline static void write(Stream& stream, const char* t)
    {
      std::string s(t);
      stream.next(s);
    }

    template<typename Stream>
    inline static void read(Stream& stream, const char* t)
    {
      std::string s;
      stream.next(s);
      t = s.c_str();
    }

    inline static uint32_t serializedLength(const char* t)
    {
      std::string s(t);
      uint32_t size = serializationLength(s);
      return size;
    }
  };
}
}

#endif //SERIALIZABLE_ANY
