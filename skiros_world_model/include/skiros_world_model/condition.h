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

#ifndef CONDITION_H
#define CONDITION_H

#include <string>
#include "skiros_world_model/world_model_interface.h"
#include <skiros_common/param_handler.h>

namespace skiros
{
namespace condition
{
/*! \brief Virtual class which define the structure of a condition.
*
*  A condition is modelled as a desired state (state_) applied on a world object (object_).
*  Since a condition is usually applied on input parameters of a skill, the condition class expects in input
*  a parameter set, and the key(s) for the parameter(s) of interest
*/
class ConditionBase
{
public:

    virtual ~ConditionBase(){}

    virtual void init(boost::shared_ptr<skiros_wm::WorldModelInterface> wm, boost::shared_ptr<skiros_common::ParamHandler> ph, bool desired_state, std::string subject_key, std::string object_key)
    {
        abstract_types_ = false;
        wm_ = wm;
        ph_ = ph;
        description_ = "Undefined";
        type_ = "";
        subject_key_ = subject_key;
        object_key_ = object_key;
        state_ = desired_state;
        if(wm_ == NULL)
            ROS_WARN("[Condition]Wm not initialized correctly");
        if(ph_ == NULL)
            ROS_WARN("[Condition]ParameterHandler not initialized correctly");
    }

    /*!
     * \brief Evaluate semantically whether the condition is true or false
     * \return
     */
    virtual bool evaluate()
	{
        return true;
	}

    /*!
     * \brief Check, using a sensing routine, if the condition is true or false
     * \return
     */
    virtual bool check()
    {
        return true;
    }

    /*!
     * \brief Set the condition to the desired state
     * \return Positive value if true, zero or negative if false
     */
    virtual bool set() = 0;

    virtual skiros_common::any getPropertyTargetValue() = 0;

    virtual std::string getPropertyType() = 0;

    skiros_wm::Element toElement()
    {
        skiros_wm::Element wm_condition(getType());
        wm_condition.addPropertyString("description",getDescription());
        wm_condition.addProperty("hasDesiredState", getDesiredState());
        wm_condition.addProperty("allowsAbstractTypes", allowsAbstractTypes());
        wm_condition.addPropertyString("hasSubject", getSubjectKey());
        wm_condition.addPropertyString("hasObject", getObjectKey());
        wm_condition.addPropertyString("hasObjectType", getObject().type());
        wm_condition.addPropertyString("hasSubjectType", getSubject().type());
        return wm_condition;
    }

    inline std::string getObjectKey(){return object_key_;}

    inline std::string getSubjectKey(){return subject_key_;}

    inline skiros_wm::Element getSubject(){return ph_->getParamValue<skiros_wm::Element>(subject_key_);}

    inline skiros_wm::Element getObject()
    {
        if(object_key_!="") return ph_->getParamValue<skiros_wm::Element>(object_key_);
        else return skiros_wm::Element();
    }

    inline bool getDesiredState(){return state_;}

    inline bool allowsAbstractTypes(){return abstract_types_;}

    inline std::string getType(){return type_;}

    inline std::string getDescription(){return description_;}

    inline void setType(std::string type){type_ = type;}
protected:
    ConditionBase(){}
    void updateSubject(skiros_wm::Element e)
    {
        ph_->specify(subject_key_, e);
        wm_->updateElement(e);
    }
    inline void setAllowAbstractTypes(bool value=true){abstract_types_ = value;}
    inline void setDescription(std::string description){description_ = description;}
    //!< The condition type as defined in the ontology
    std::string type_;
    std::string description_;
    //!< Object key in the ParamHandler
    std::string object_key_;
    //!< Object key in the ParamHandler
    std::string subject_key_;
    //!< Desired state
    bool state_;
    //!< True if allow to have in input abstract types
    bool abstract_types_;
    //!< The constraint is applied on objects holded in an external param handler
    boost::shared_ptr<skiros_common::ParamHandler> ph_;
    //!< Interface to the ontology\world model
    boost::shared_ptr<skiros_wm::WorldModelInterface> wm_;
};

template<class T>
/*!
 * \brief The ConditionProperty is a condition applied on a object's property. It usually expects in input one object
 */
class ConditionProperty : public ConditionBase
{
    public:

        ConditionProperty()
        {
            setType("ConditionProperty");
            setPropertyType("");
            setPropertyValue(T());
        }

        ~ConditionProperty(){}

        virtual bool evaluate()
        {
            updateDescription();
            skiros_wm::Element subject = getSubject();
            if(subject.id()<0) return false;
            std::string temp = getPropertyType();
            //FINFO(subject.printState());
            if(!subject.hasProperty(temp)) {return !getDesiredState();}
            skiros_common::Param property = subject.properties(temp);
            for(int i=0;i<property.size();i++)
            {
                if(property.getValue<T>(i)==property_value_)
                {
                    return getDesiredState();
                }
            }
            return !getDesiredState();
        }

        virtual bool set()
        {
            updateDescription();
            skiros_wm::Element subject = getSubject();
            if(subject.id()<0) return false;
            subject = wm_->getElement(subject.id());
            if(getDesiredState())
            {
                //True
                if(!subject.hasProperty(getPropertyType()))
                    subject.addProperty(getPropertyType(), property_value_);
                else
                {
                    if(subject.properties(getPropertyType()).size()>1)
                        subject.addProperty(getPropertyType(), property_value_);
                    else
                        subject.properties(getPropertyType()).setValue(property_value_);
                }
            }
            else
            {
                //False
                if(subject.hasProperty(property_type_))
                {
                    if(subject.properties(property_type_).size()>1)
                    {
                        for(int i=0;i<subject.properties(property_type_).size();i++)
                        {
                            if(subject.properties(property_type_).getValue<T>(i)==property_value_)
                                subject.properties(getPropertyType()).erase(i);
                        }
                    }
                    else {subject.removeProperty(property_type_);}
                }
            }
            updateSubject(subject);
            return true;
        }


        skiros_common::any getPropertyTargetValue(){return skiros_common::any(property_value_);}
        std::string getPropertyType(){return property_type_;}
protected:
        void updateDescription()
        {
            skiros_wm::Element subject = getSubject();
            if(subject.id()>=0)
            {
                std::stringstream ss;
                ss  << "[" << getType() <<  "] Check property: ";
                if(!getDesiredState()) ss << "Not ";
                ss << property_type_ << " for object: " << subject.printState("", false);
                description_ = ss.str();
            }
            else
            {
                description_ = "Condition on property " + property_type_;
            }
        }
        void setPropertyType(std::string type){property_type_ = type;}
        void setPropertyValue(T value){property_value_ = value;}
        std::string property_type_;
        T property_value_;
};

/*!
 * \brief The ConditionRelation is a condition applied on a relation between two objects. It usually expects in input two objects
 */
class ConditionRelation : public ConditionBase
{
    public:

        ConditionRelation()
        {
            setType("ConditionRelation");
            setRelationType("");
        }

        ~ConditionRelation(){}

        virtual bool evaluate()
        {
          updateDescription();
          //Extract the ID values from the parameter handler
          skiros_wm::Element subject = getSubject();
          if(subject.id()<0) return false;
          skiros_wm::Element object = getObject();
          if(object.id()<0) return false;
          //Use the IDs to query the WM about the object location
          //FINFO("Query " << subject.id() << relation_type_ << object.id());
          skiros_wm::RelationsVector v = wm_->queryRelation(subject.id(), relation_type_, object.id());
          //If the query return a match means that the object is contained into location
          if(v.size()>0) return getDesiredState();
          else return !getDesiredState();
        }

        virtual bool set()
        {
            updateDescription();
            skiros_wm::Element subject = getSubject();
            if(subject.id()<0) return false;
            skiros_wm::Element object = getObject();
            if(object.id()<0) return false;
            if(wm_->setRelation(subject.id(), relation_type_, object.id(), getDesiredState()))return true;
            else return false;
        }

        skiros_common::any getPropertyTargetValue(){return ph_->getParamMap().at(object_key_).getValue();}
        std::string getPropertyType(){return relation_type_;}
protected:
        void updateDescription()
        {
            skiros_wm::Element object = getObject();
            skiros_wm::Element subject = getSubject();
            if(subject.id()>=0 && object.id() >=0)
            {
                std::stringstream ss;
                ss  << "[" << getType() <<  "] Check relation: ";
                if(!getDesiredState()) ss << "Not ";
                ss << subject.printState("", false) << "->" << relation_type_ << "->" << object.printState("", false);
                description_ = ss.str();
            }
            else
            {
                description_ = "Condition on relation " + relation_type_;
            }
        }

        void setRelationType(std::string type){relation_type_ = type;}
        std::string relation_type_;
};


boost::shared_ptr<ConditionBase> loadCondition(boost::shared_ptr<skiros_wm::WorldModelInterface> wm, boost::shared_ptr<skiros_common::ParamHandler> ph, std::string condition_name, bool desired_state, std::string subject, std::string object="");

}
}

#endif // CONDITION_H
