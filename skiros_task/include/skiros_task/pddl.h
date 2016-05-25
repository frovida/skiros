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

#ifndef PDDL_H
#define PDDL_H

#include <skiros_task/type_manager.h>
#include <skiros_world_model/world_model_interface.h>

namespace skiros_wm
{
namespace pddl
{

/*
e.g. (Predicate ?param_id1 ?param_id2)
*/
struct UngroundPredicate{
    std::string predicate_name;
    std::vector<std::string> parameter_ids;
    std::vector<std::string> parameter_types;
    bool forall = false;
    std::string to_pddl(){
        std::string s;
        s += "(" + predicate_name;
        for(unsigned i = 0; i < parameter_ids.size(); ++i){
            s += " ?" + parameter_ids.at(i);
        }
        s += ")";
        return s;
    }
    std::string to_forall_pddl(int var_pos){
        std::string s;
        s += "(" + predicate_name;
        for(unsigned i = 0; i < parameter_ids.size(); ++i){
            if(i == var_pos)
                s += " ?r";
            else
                s += " ?" + parameter_ids.at(i);
        }
        s += ")";
        return s;   
    }
};

struct PM_Skill{
    std::string skill_name;
    std::vector<std::string> parameter_ids;
    std::vector<bool> param_in_skiros;//does the paramter need to be returned to skiros?
    std::vector<std::string> parameter_types; //types
    std::vector<UngroundPredicate> negative_preconditions;
    std::vector<UngroundPredicate> preconditions; //refers to parameters list
    std::vector<UngroundPredicate> add_effects; //
    std::vector<UngroundPredicate> del_effects;
    bool measure_distance = false;
    bool require_parts = false;

    std::string to_pddl(){
        std::stringstream s;
        //open
        s << "(:durative-action " << skill_name << "\n";
        //parameters
        s << "\t:parameters (";
        for(unsigned i = 0; i < parameter_ids.size(); ++i){
            s << " ?" << parameter_ids.at(i) << " - ";  
            s << parameter_types.at(i);
        }   
        s << ")\n";
        //duration
        if(measure_distance){
            s << "\t:duration (= ?duration (distance ?preTargetLocation ?TargetLocation))\n";          
        }
        else
            s << "\t:duration (= ?duration 1)\n";
        //precondition
        s << "\t:condition (and\n";
        for(UngroundPredicate up : preconditions){
            s << "\t\t(at start " << up.to_pddl() << ")\n";
        }
        for(UngroundPredicate up : negative_preconditions){
            s << "\t\t(at start (not " << up.to_pddl() << "))\n";
        }
        if(require_parts)
            s << "\t\t(at start (> (parts ?Container) 0) )\n";
        s << "\t)\n";
        //effect
        s << "\t:effect (and\n";
        //-ve effs
        for(UngroundPredicate up : del_effects){
            if(up.forall){
                s << "\t\t(at start (forall (?r - Agent) (not " << up.to_forall_pddl(0) << ")))\n";
            }
            else
                s << "\t\t(at start (not " << up.to_pddl() << "))\n";
        }
        //+ve effs
        for(UngroundPredicate up : add_effects){
            if(up.forall){
                s << "\t\t(at end (forall (?r - Agent) " << up.to_forall_pddl(0) << "))\n";
            }
            else
                s << "\t\t(at end " << up.to_pddl() << ")\n";
        }
        if(require_parts){
            s << "\t\t(at end (decrease (parts ?Container) 1))\n";
        }
        //s << "\t\t(increase (total-cost) 1)\n";
        s << "\t)\n";
        //close
        s << ")\n";
        return s.str();
    }
};

struct GroundSkill{
    std::string skill_name;
    std::vector<std::string> obj_names;
    std::string to_str(){
        std::string s;
        s += skill_name + " ";
        for(std::string o : obj_names)
            s += o + " ";
        return s;
    }
};

class PddlProblem
{
    //Need to know common name used for Robot and its type.
    const std::string robotParameterName = "Robot";
    const std::string robotTypeName = "Agent";

    //In STAMINA need to specify that Manipulatable have abstract types
    //that drive uses sequnce numbers
    //and  that pick uses part numbers
    const bool STAMINA = true;    
    const std::string hasAbstractTypes = "Manipulatable";
    const std::string requireSequenceNumbers = "drive";
    const std::string requirePartNumbers = "pick";
    //SkiROS Information
    typedef std::map<std::string, Element> SkillsElementMap;
    typedef std::pair<std::string, Element> SkillsElementPair;
    SkillsElementMap e_skills_;
    boost::shared_ptr<WorldModelInterface> wm_;
    //PDDL Level Information
    Type_Manager type_manager;
    std::vector<PM_Skill> skills;
    //Return
    std::vector<GroundSkill> plan;
public:
    PddlProblem(){}
    ~PddlProblem(){}

    boost::shared_ptr<WorldModelInterface> getWorldHandle(){return wm_;}
    void initWorldInterface(boost::shared_ptr<WorldModelInterface> wm){wm_ = wm; type_manager.initWorldInterface(wm);}

    void dumpAll();
    void resetAll();

    void initDomain();
    void setGoal(std::vector<skiros_wm::Element> conditions, std::vector<std::string> pddl_goals);
    void initProblem();
    void outputPDDL();
    std::vector<Element> callPlanner();

private:
    PDDL_Type getType(std::string s);
    //void setSuperType(PDDL_Type t, std::string current_type);
    std::string addPredicate(Element e);
    void parseSkill(Element skill);
    UngroundPredicate parseUngroundPredicate(Element condition);

    void printDomainFile();
    void printProblemFile();

    std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
    std::vector<std::string> split(const std::string &s, char delim);

    std::vector<Element> getRobots();
    std::vector<Element> getRobotSkills(Element robot);
    std::vector<Element> getPreConditions(Element skill);
    std::vector<Element> getPostConditions(Element skill);

    //helper methods
    bool isSpatialRelation(UngroundPredicate up);
    bool isRelatedSpatialRelation(UngroundPredicate up1, UngroundPredicate up2);
};

}
}


#endif // PDDL_H
