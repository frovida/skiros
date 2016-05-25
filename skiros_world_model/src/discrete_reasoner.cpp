#include "skiros_world_model/discrete_reasoner.h"
#include "skiros_config/declared_uri.h"

using namespace skiros_config::owl;

namespace skiros_wm
{

void DiscreteReasoner::addProperties(skiros_wm::Element& e)
{
    //Mark the presence of the type of reasoner
    if(!e.hasProperty(data::DiscreteReasoner))
        e.addProperty(data::DiscreteReasoner, getType());
    else
    {
        if(e.properties(data::DiscreteReasoner).find(getType())<0)
            e.properties(data::DiscreteReasoner).push_back(getType());
    }
    //Mark the name of the reasoner
    if(!e.hasProperty(getType()))
        e.addProperty(getType(), getName());
    else
    {
        if(e.properties(getType()).find(getName())<0)
            e.properties(getType()).push_back(getName());
    }
    onAddProperties(e);
}
}
