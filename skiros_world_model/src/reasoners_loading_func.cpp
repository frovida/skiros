#include "skiros_world_model/reasoners_loading_func.h"
#include <boost/foreach.hpp>
#include <pluginlib/class_loader.h>
#include <skiros_world_model/plugin_loading_func.h>

namespace skiros_wm
{

std::map<std::string, ReasonerPtrType> reasoners;

ReasonerPtrType getDiscreteReasoner(std::string name)
{
    ReasonerPtrType to_ret;
    if(reasoners.find(name)!=reasoners.end())
        to_ret = reasoners.at(name);
    else
        to_ret = skiros::loadPlugin<skiros_wm::DiscreteReasoner>("skiros_world_model", "skiros_wm::DiscreteReasoner", name);
    if(to_ret!=NULL)
    {
        reasoners.insert(std::pair<std::string, ReasonerPtrType>(name, to_ret));
    }
    return to_ret;
}

std::vector<std::string> getAvailableReasoners()
{
    return skiros::getAvailablePlugins<skiros_wm::DiscreteReasoner>("skiros_world_model", "skiros_wm::DiscreteReasoner");
}
}
