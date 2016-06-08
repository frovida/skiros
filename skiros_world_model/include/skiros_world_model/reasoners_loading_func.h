#ifndef REASONER_LOADING_FUNC_H
#define REASONER_LOADING_FUNC_H

#include "skiros_world_model/discrete_reasoner.h"

namespace skiros_wm
{
typedef boost::shared_ptr<skiros_wm::DiscreteReasoner> ReasonerPtrType;
/*!
 * \brief Load a singleton DiscreteReasoner plugin
 * \param name the plugin id as defined in the .xml
 * \return shared pointer to the reasoner's class
 */
ReasonerPtrType getDiscreteReasoner(std::string name = "");
template<typename T>
/*!
 * \brief Load a DiscreteReasoner plugin and downcast it to a class T
 * \param name name the plugin id as defined in the .xml
 * \return shared pointer to the reasoner's instance
 */
boost::shared_ptr<T> getDiscreteReasonerSpecialized(std::string name = "")
{
    return boost::dynamic_pointer_cast<T>(getDiscreteReasoner(name));
}

//! \brief Get the list of available reasoners
std::vector<std::string> getAvailableReasoners();
}


#endif //REASONER_LOADING_FUNC_H
