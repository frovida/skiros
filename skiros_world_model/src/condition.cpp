#include <skiros_world_model/condition.h>
#include <skiros_world_model/plugin_loading_func.h>

namespace skiros
{
namespace condition
{
boost::shared_ptr<ConditionBase> loadCondition(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                                               boost::shared_ptr<skiros_common::ParamHandler> ph,
                                               std::string condition_name,
                                               bool desired_state,
                                               std::string subject,
                                               std::string object)
{
    boost::shared_ptr<ConditionBase> to_ret = skiros::loadPlugin<ConditionBase>("skiros_skill", "skiros::condition::ConditionBase", condition_name);
    if(to_ret==NULL) return to_ret;
    to_ret->init(wm, ph, desired_state, subject, object);
    to_ret->setType(condition_name);
    return to_ret;
}
}
}
