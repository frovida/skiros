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
#ifndef MODULE_CORE_H
#define MODULE_CORE_H

#include <string>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <ros/ros.h>
#include "skiros_world_model/world_model_interface.h"
#include "skiros_world_model/condition.h"
#include "skiros_world_model/module_states.h"
#include "skiros_common/param_handler.h"

namespace skiros
{
    /*!
     * \brief The Progress struct describe the progress of a module execution
     */
    struct Progress
    {
        Progress(): id(0), description(""){}
        Progress(int id_in, std::string description_in): id(id_in), description(description_in){}
        int id;
        std::string description;
    };

    /*!
     * \brief The ProgressAndState struct groups progress and state for visualization
     */
    struct ProgressAndState
    {
        Progress progress;
        state::StateType state;
    };

    /*! @brief The ModuleCore is the base virtual class for every skiros module (primitive, skill, learning module, etc.)
     *
     *  Every module is characterized to have a State (uninitialized, initialized, running, terminated, etc.) and a Progress (different for every module).
     *  A module is initialized with the init() function. After that, its execution can be started and preempted.
     */
    class ModuleCore
    {
    public:

        virtual ~ModuleCore() {}

        //---------- EXECUTION FUNCTIONS ----------------
        //! \brief specialized preempt routine
        virtual void onPreempt(){}
        //! \brief main execution routine
        virtual int execute() = 0;
        //! \brief checks that the module is initialized correctly, then triggers the execution
        virtual void start();
        //! \brief initialize the params, then calls start()
        inline void start(skiros_common::ParamMap input_param_map, std::string controller)
        {
            setParams(input_param_map);
            start(controller);
        }
        void start(std::string controller) {controller_ = controller; start();}
        //! \brief stop the execution of a skill
        void preempt(std::string reason)
        {
            this->onPreempt();
            this->setState(skiros::state::preempted, false);
            this->setProgress(reason);
        }

        //void pause();//TODO
        //! \brief Pass to the skill the ROS node handler and the interface to the world model
        virtual void init(boost::shared_ptr<ros::NodeHandle> nh, boost::shared_ptr<skiros_wm::WorldModelInterfaceS> world_model)
        {
            nh_ = nh;
            world_model_ = world_model;
            this->setResultCode(0);
        }
        //! \brief True after init is called succesfully
        inline const bool isInitialized() { return (skiros::state::uninitialized!=this->getState());}

        //---------- DEFINITION FUNCTIONS ----------------
        inline const std::string skillType()  /*__attribute__ ((deprecated))*/  const  { return type_; }
        inline const std::string moduleType() const  { return type_; }
        inline const std::string description() const	{  return description_;	}
        inline const std::string version() const { return version_;	}

        //---------- STATE FUNCTIONS ----------------
        //! \brief Check if the module is in a precise state
        inline bool isPreempted()
        {
            return skiros::state::preempted==this->getState();
        }
        inline bool isRunning()
        {
            return skiros::state::running==this->getState();
        }
        //! \brief Wait the module to go in a precise state
        void waitState(skiros::state::StateType state)
        {
            boost::mutex::scoped_lock lock(state_mux_);
            while (state!=this->getState())
                state_cond_.wait(lock); //TODO: wait timeout with chrono
        }
        //! \brief Wait for any state change in the module
        skiros::state::StateType waitStateChange()
        {
            boost::mutex::scoped_lock lock(state_mux_);
            while (!state_changed_)
                state_cond_.wait(lock); //TODO: wait timeout with chrono
            return state_;
        }
        //! \brief Return true if the state changed since last call to "getState"
        inline bool isStateChanged() {return state_changed_;}
        //! \brief Get the module state
        skiros::state::StateType getState()
        {
            boost::mutex::scoped_lock lock(state_mux_);
            state_changed_ = false;
            return state_;
        }

        ProgressAndState waitOutput()
        {
            boost::mutex::scoped_lock lock(state_mux_);
            while (out_list_.size()<=0)
                state_cond_.wait(lock);
            ProgressAndState to_ret = out_list_.back();
            out_list_.pop_back();
            return to_ret;
        }

        //---------- OTHER FUNCTIONS ----------------
        std::string getController(){return controller_;}
        //! \brief Get the parameter handler of the skill
        inline skiros_common::ParamHandler getParamHandlerCopy(){return *ph_ptr_;}
        //! \brief Get the execution result code
        inline int executionResult() {return return_code_;}
        /*!
         * \brief set the parameters (it is mandatory to pass the parameters to the skill before execution, prediction, etc.). The specification process is handled by the skill manager
         * \param input_param_map
         * \return true if all params are specified, false otherwise
         */
        bool setParams(skiros_common::ParamMap input_param_map);
        bool setParams(skiros_common::ParamHandler ph);

        //! \brief Get the skill progress
        Progress getProgress()
        {
            boost::mutex::scoped_lock lock(state_mux_);
            state_changed_ = false;
            return progress_;
        }

    protected:
        ModuleCore(): progress_id_(0), state_changed_(false), param_are_specified_(false), state_(skiros::state::uninitialized), type_("Undefined"), description_("Undefined"), version_("Undefined"), skill_is_active_(false)
        {
            ph_ptr_.reset(new skiros_common::ParamHandler());
        }

        //------------ SET -----------------
        //! \brief Set the type
        inline void setSkillType(std::string name)  /*__attribute__ ((deprecated))*/ { type_ = name; }
        inline void setModuleType(std::string name) { type_ = name; }
        //! \brief Set the description
        inline void setDescription(std::string description) { description_ = description; }
        //! \brief Set the version
        inline void setVersion(std::string version) { version_ = version; }
        //! \brief Set the state
        void setState(skiros::state::StateType state, bool notify=true)
        {
            boost::mutex::scoped_lock lock(state_mux_);
            state_ = state;
            state_changed_ = true;
            if(notify) pushOutProgress();
        }
        //! \brief Set the result code
        inline void setResultCode(int value) { return_code_ = value; }
        //! \brief Set the progress, automatically increase the id (stream version)
        //TODO: stringstream doesn't work in all cases, better istream, but how to do?
        inline void setProgressStream(std::ostringstream description){setProgress(description.str());}
        //! \brief Set the progress, automatically increase the id
        inline void setProgress(std::string description){setProgress(Progress(++progress_id_, description)); }
        //! \brief Set the progress
        inline void setProgress(int id, std::string description)	{ progress_id_=id; setProgress(Progress(progress_id_, description)); }
        //! \brief Set the progress
        void setProgress(Progress progress)
        {
            boost::mutex::scoped_lock lock(state_mux_);
            progress_ = progress;
            pushOutProgress();
        }
        //! \brief Push the progress in the output list
        void pushOutProgress()
        {
            ProgressAndState out;
            out.state = state_;
            out.progress = progress_;
            out_list_.push_front(out);
            state_cond_.notify_all();
        }

        //! \brief Initialize the progress
        void initProgress();
        //------------ GET -----------------
        //! \brief Interface with the parameter set
        boost::shared_ptr<skiros_common::ParamHandler> getParamHandler() { return ph_ptr_; }
        boost::shared_ptr<skiros_common::ParamHandler> getParamHandle() { return ph_ptr_; }
        //! \brief Interface with the skiros world model
        boost::shared_ptr<skiros_wm::WorldModelInterfaceS> getWorldHandler() { return world_model_; }
        boost::shared_ptr<skiros_wm::WorldModelInterfaceS> getWorldHandle() { return world_model_; }
        //! \brief Interface with the ROS network
        boost::shared_ptr<ros::NodeHandle> getNodeHandler() { return nh_; }
        boost::shared_ptr<ros::NodeHandle> getNodeHandle() { return nh_; }

        //------------ MEMBER VARIABLEs -----------------
        //!< Current controller of the module
        std::string controller_;
        //!< True when all parameters have been specified
        bool param_are_specified_;
        //!< Holds the input parameters set
        boost::shared_ptr<skiros_common::ParamHandler> ph_ptr_;
        //!< Pointers to interface to the world model
        boost::shared_ptr<skiros_wm::WorldModelInterfaceS> world_model_;
        //!< Pointers to interface to the ROS system
        boost::shared_ptr<ros::NodeHandle>  nh_;
        //!< Used to keep track of the skill execution progress. Used also to continue the execution after a skill pause() command
        Progress progress_;
        skiros::state::StateType state_;
        int progress_id_;
        bool state_changed_;
        //!< A list of outputs TODO: add the timestamp
        std::list<ProgressAndState> out_list_;
        //!< Result of the skill execution
        int return_code_;
        boost::mutex state_mux_;
        boost::condition_variable state_cond_;
        //!< Modules info variables
        std::string type_;
        std::string description_;
        std::string version_;
        bool skill_is_active_;
    };
}
#endif // MODULE_CORE_H
