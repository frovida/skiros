#include<skiros_skill/module_monitor.h>

namespace skiros
{

bool SAVE_LOG=true;
boost::mutex LOG_MUX;

void ModuleMonitor::printStatus(ProgressAndState out)
{
    skiros_msgs::ModuleStatus msg;
    Progress prog = out.progress;
    state::StateType state = out.state;
    std::ostringstream output;
    if (prog.id == 0) output << "\033[1;32m";  // Bold green
    else if (state == state::preempted) output << "\033[0;31m"; // Red
    else if (state == state::error) output << "\033[1;31m"; // Bold red
    else if (state == state::running) output << "\033[0;32m"; // green
    else if (state == state::terminated) output << "\033[1;32m"; // Bold green
    output << module_name_ << "[" << state::StateStr[state]
            << "]: "  << "[" << prog.id << "]" <<  prog.description;
    output << "\033[0m";
    FINFO(output.str());
    //TODO: in parameters on start
    //FINFO(moduleType() << "[" << StateStr[state]
    //        << "]: "  << "[" << prog.id << "]" <<  prog.description);
    msg.header.stamp = ros::Time::now();
    msg.module.type = module_->moduleType();
    msg.module.name = module_name_;
    msg.status = state::StateStr[state];
    if(state == state::started)
    {
        start_time = ros::Time::now();
    }
    if(state == state::preempted || state == state::error || state == state::terminated)
    {
        if(!is_skill_)
        {
            skiros::ModuleBase * temp = static_cast<skiros::ModuleBase *>(module_.get());
            msg.module.parameters_out = skiros_common::utility::serializeParamMap(temp->getResultParamHandlerCopy().getParamMap());
        }
        if(SAVE_LOG)
        {
            skiros_common::ParamMap onlineParams = module_->getParamHandlerCopy().getParamMapFiltered(skiros_common::online);
            std::stringstream ss;
            ss << module_->getController() << " "
               << module_name_  << " "
               << prog.id << " "
               << "'" << prog.description << "'"
               << " " << (ros::Time::now()-start_time).toSec() << " no ";
            for(auto pair : onlineParams)
            {
                skiros_common::Param p = pair.second;
                if(p.type()==typeid(skiros_wm::Element))
                {
                    ss << "'" << p.key() << ":" << p.getValue<skiros_wm::Element>().type() << "' ";
                }
            }
            ss << "\n" ;
            std::string save_path = skiros_common::utility::getSkirosSaveDirectory() + "all_modules_log.txt";
            FILE * log_;
            LOG_MUX.lock();
            log_ = fopen(save_path.c_str(), "a");
            if(!log_)
                FERROR("[ModuleMonitor::start] Failed to open log file.");
            if(log_)
            {
                FINFO("SAVING: " << ss.str());
                fputs(ss.str().c_str(), log_);
                fclose(log_);
            }
            LOG_MUX.unlock();
        }
    }
    //msg.module.parameters_in = module_->getParamHandle().module_->getParamMap();
    msg.progress_code = prog.id;
    msg.progress_description = prog.description;
    state_pub_.publish(msg);
}
}
