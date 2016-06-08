#include "skiros_world_model/discrete_reasoner.h"
#include "skiros_config/declared_uri.h"

using namespace skiros_config::owl;

namespace skiros_wm
{

void DiscreteReasoner::addProperties(skiros_wm::Element& e)
{
    //Mark the object with the name of the reasoner
    if(!e.hasProperty(data::DiscreteReasoner))
        e.addProperty(data::DiscreteReasoner, getName());
    else
    {
        if(e.properties(data::DiscreteReasoner).find(getName())<0)
            e.properties(data::DiscreteReasoner).push_back(getName());
    }
    onAddProperties(e);
}

void DiscreteReasoner::removeProperties(skiros_wm::Element& e)
{
    e.removeProperty(data::DiscreteReasoner);
    onRemoveProperties(e);
}
}
