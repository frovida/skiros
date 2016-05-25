/********************************************************************
 ***                                                              ***
 ***  PKS - PLANNING WITH KNOWLEDGE AND SENSING                   ***
 ***                                                              ***
 ***  Copyright (C) 2014  Ron Petrick                             ***
 ***  All Rights Reserved                                         ***
 ***                                                              ***
 ********************************************************************
 ***  THIS SOFTWARE IS NOT OPEN SOURCE. DO NOT DISTRIBUTE.        ***
 ********************************************************************
 ***  This software is not open source nor freely distributable.  ***
 ***  This source file is provided for the sole purpose of        ***
 ***  building a binary executable file on platforms for which a  ***
 ***  binary file has not otherwise been provided. This source    ***
 ***  file should not be changed, modified, or otherwise          ***
 ***  redistributed in any way.                                   ***
 ********************************************************************
 ***  Author:  Ron Petrick                                        ***
 ***  E-mail:  rpetrick@gmail.com                                 ***
 ***  Updated: 2014-10-09                                         ***
 ***  Version: 1.9.4                                              ***
 ********************************************************************
 ***  FILE:    pks-api.h                                          ***
 ********************************************************************/

#ifndef PLANNER_INTERFACE_H
#define PLANNER_INTERFACE_H

#include <string>
#include <vector>

namespace skiros {
namespace planner {


/*!
 * \brief The StateProperty struct
 */
struct StateProperty {
    std::string name;
    std::vector<std::string> args;
    bool sign;
    std::string value;
};


/*!
 * \brief The PlanStep struct
 */
struct PlanStep {
    std::string name;
    std::string type;
    std::vector<std::string> args;
};

/*!
 * \brief This virtual class define the interface for every planner
 */
class PlannerBase {
protected:
        PlannerBase(){}
public:
        virtual ~PlannerBase(){}

        virtual void        reset() = 0;
        virtual bool        setPlannerProperty(std::string, std::string)= 0;
        virtual std::string getPlannerProperty(std::string)= 0;

        virtual void        clearDomain()= 0;
        virtual void        clearDomainSymbols()= 0;
        virtual void        clearDomainActions()= 0;
        virtual void        clearDomainProblems()= 0;
        virtual void        clearDomainInitialState()= 0;

        virtual bool        loadDomain(std::string)= 0;
        virtual bool        loadDomainSymbols(std::string)= 0;
        virtual bool        loadDomainActions(std::string)= 0;
        virtual bool        loadDomainProblems(std::string)= 0;
        virtual bool        loadDomainProblemGoal(std::string, std::string)= 0;
        virtual bool        loadDomainProblemUpdateRules(std::string, std::string)= 0;
        virtual bool        loadDomainProblemMaintenanceRules(std::string, std::string)= 0;
        virtual bool        loadDomainProblemInitialState(std::string, std::string)= 0;
        virtual bool        loadDomainProblemInitialStateFacts(std::string, std::string)= 0;
        virtual bool        loadDomainInitialState(std::string)= 0;
        virtual bool        loadDomainInitialStateFacts(std::string)= 0;

        virtual bool        defineDomain(std::string)= 0;
        virtual bool        defineDomainSymbols(std::string)= 0;
        virtual bool        defineDomainTypeSymbol(std::string)= 0;
        virtual bool        defineDomainConstantSymbol(std::string, std::string)= 0;
        virtual bool        defineDomainActions(std::string)= 0;
        virtual bool        defineDomainProblems(std::string)= 0;
        virtual bool        defineDomainProblemGoal(std::string, std::string)= 0;
        virtual bool        defineDomainProblemUpdateRules(std::string, std::string)= 0;
        virtual bool        defineDomainProblemMaintenanceRules(std::string, std::string)= 0;
        virtual bool        defineDomainProblemInitialState(std::string, std::string)= 0;
        virtual bool        defineDomainProblemInitialStateFacts(std::string, std::string)= 0;
        virtual bool        defineDomainProblemInitialStateFacts(std::string, std::vector<StateProperty> &)= 0;
        virtual bool        defineDomainInitialState(std::string)= 0;
        virtual bool        defineDomainInitialStateFacts(std::string)= 0;
        virtual bool        defineDomainInitialStateFacts(std::vector<StateProperty> &)= 0;

        virtual void        clearPlan()= 0;
        virtual void        resetPlan()= 0;
        virtual bool        buildPlan()= 0;
        virtual bool        getPlan(std::vector<PlanStep> &)= 0;
        virtual bool        getPlanState(std::vector<StateProperty> &)= 0;
        virtual bool        getPlanAction(PlanStep &)= 0;
        virtual bool        isPlanDefined()= 0;
        virtual bool        isEndOfPlan()= 0;
        virtual void        advancePlan()= 0;
        virtual bool        setPlanProblem(std::string)= 0;

        virtual bool        checkPlanActionPreconditions(std::vector<StateProperty> &)= 0;
        virtual bool        getPlanActionEffects(std::vector<StateProperty> &,
                                         std::vector<StateProperty> &,
                                         std::vector<StateProperty> &)= 0;
        virtual bool        getPlanActionEffectsState(std::vector<StateProperty> &,
                                              std::vector<StateProperty> &)= 0;
        virtual bool        getPlanInitialState(std::vector<StateProperty> &)= 0;
        virtual bool        getDomainInitialState(std::vector<StateProperty> &)= 0;
        virtual bool        getDomainProblemInitialState(std::vector<StateProperty> &)= 0;
        virtual bool        getStateDifference(std::vector<StateProperty> &,
                                       std::vector<StateProperty> &,
                                       std::vector<StateProperty> &,
                                       std::vector<StateProperty> &)= 0;

        virtual std::string getStatePropertyLabel(StateProperty &)= 0;
        virtual std::string getPlanStepLabel(PlanStep &)= 0;
};

}
}
#endif //PLANNER_INTERFACE_H

