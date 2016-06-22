#include <skiros_task/planner_interface.h>
#include <skiros_config/declared_uri.h>
#include <skiros_common/param_handler.h>
#include <skiros_world_model/condition.h>
#include <ros/package.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>


using namespace std;
using namespace skiros_config::owl;

namespace skiros_wm
{
namespace pddl
{

void PlannerInterface::resetAll(){
    planner_model.reset();
    skills.clear();
    plan.clear();
}

//Resets the domain and then parses all existing skills that belong to at least one robot
void PlannerInterface::initDomain()
{
    if(getWorldHandle()==NULL) return;
    resetAll();

    ROS_INFO("Getting Skills");
    //GetSkills (from robots)
    std::vector<Element> robots = getRobots();
    int nrRobots = robots.size();
    for(Element e : robots)
    {
        //FINFO(e.printState("", false));
        std::vector<Element> skills = getRobotSkills(e);
        for(Element s : skills)
        {
            //FINFO(s.printState("", false));
            parseSkill(s);
            //Add can_x ground predicates for each object capable of performing skill s
            std::vector<std::string> args;
            args.push_back(e.toUrl());
            std::stringstream ss;
            ss << "can_" << s.label();
            planner_model.addGroundPredicate(ss.str(), args);
            if(e_skills_.find(s.label())==e_skills_.end())e_skills_.insert(SkillsElementPair(s.label(), s));
        }
    }

    ROS_INFO("Setting Type Heirarchy");
    planner_model.setTypeHeirarchy();

    //add can_x so only robot with skill can use it and also add missing preconditions/effects
    for(unsigned i = 0; i < skills.size(); ++i){
        PM_Skill s = skills.at(i);

        //Add can_x predicate
        planner_model.addPredicate("can_" + s.skill_name, {robotParameterName}, false);

        //First Add can_x to precondition
        UngroundPredicate can_skill;
        can_skill.predicate_name = "can_" + s.skill_name;
        can_skill.parameter_ids.push_back(robotParameterName);
        can_skill.parameter_types.push_back(robotTypeName);
        skills.at(i).preconditions.push_back(can_skill);

        //For any spatial relation in add effects, make sure that tree property is preserved
        //by adding preconditions and del effectes to remove old spatial relations.
        //Check each add effect (that is a spatial relation) has related del/pre
        for(UngroundPredicate up_add : s.add_effects){
            if(isSpatialRelation(up_add)){
                //First check pres and add precondition if needed
                bool has_pre_ready = false;
                bool has_del_ready = false;
                UngroundPredicate pre;
                UngroundPredicate del;
                for(UngroundPredicate up_pre : s.preconditions){
                    if(isRelatedSpatialRelation(up_pre, up_add)){
                        pre = up_pre;
                        has_pre_ready = true;
                    }
                }
                for(UngroundPredicate up_del : s.del_effects){
                    if(isRelatedSpatialRelation(up_del, up_add)){
                        del = up_del;
                        has_del_ready = true;
                    }
                }
                if(!has_pre_ready && !has_del_ready){
                    //needs new pre and del and must add new param
                    UngroundPredicate up_new_pre;
                    up_new_pre.predicate_name = up_add.predicate_name;
                    up_new_pre.parameter_ids.push_back(up_add.parameter_ids.at(0));
                    up_new_pre.parameter_ids.push_back("pre" + up_add.parameter_ids.at(1));
                    up_new_pre.parameter_types.push_back(up_add.parameter_types.at(1));
                    skills.at(i).parameter_ids.push_back("pre" + up_add.parameter_ids.at(1));
                    skills.at(i).parameter_types.push_back(up_add.parameter_types.at(1));
                    skills.at(i).param_in_skiros.push_back(false);
                    skills.at(i).preconditions.push_back(up_new_pre);
                    skills.at(i).del_effects.push_back(up_new_pre);

                    //Specific for drive skill with for multiple connected robots
                    if(STAMINA){
                        std::size_t found = s.skill_name.find("drive");
                        if(found != std::string::npos && nrRobots >  1 ){
                            std::string newr = "r";
                            UngroundPredicate nup_neg_pre;
                            UngroundPredicate nup_del;
                            UngroundPredicate nup_add;
                            nup_neg_pre.predicate_name = "can_" + s.skill_name;
                            nup_del.predicate_name = up_add.predicate_name;
                            nup_add.predicate_name = up_add.predicate_name;
                            nup_neg_pre.parameter_ids.push_back(newr);
                            nup_neg_pre.parameter_types.push_back(robotTypeName);
                            nup_del.parameter_ids.push_back(newr);
                            nup_del.parameter_types.push_back(robotTypeName);
                            nup_del.parameter_ids.push_back("pre" + up_add.parameter_ids.at(1));
                            nup_del.parameter_types.push_back(up_add.parameter_types.at(1));
                            nup_add.parameter_ids.push_back(newr);
                            nup_add.parameter_types.push_back(robotTypeName);
                            nup_add.parameter_ids.push_back(up_add.parameter_ids.at(1));
                            nup_add.parameter_types.push_back(up_add.parameter_types.at(1));
                            skills.at(i).parameter_ids.push_back(newr);
                            skills.at(i).parameter_types.push_back(robotTypeName);
                            skills.at(i).param_in_skiros.push_back(false);
                            skills.at(i).negative_preconditions.push_back(nup_neg_pre);
                            skills.at(i).del_effects.push_back(nup_del);
                            skills.at(i).add_effects.push_back(nup_add);
                        }
                    }
                }
                else if(has_del_ready && !has_pre_ready){
                    //needs precondition to match del
                    UngroundPredicate up_new_pre;
                    up_new_pre.predicate_name = del.predicate_name;
                    for(std::string s : del.parameter_ids){
                        up_new_pre.parameter_ids.push_back(s);
                    }
                    for(std::string s : del.parameter_types){
                        up_new_pre.parameter_types.push_back(s);
                    }
                    skills.at(i).preconditions.push_back(up_new_pre);
                }
                else if(has_pre_ready && !has_del_ready && !s.require_parts){
                    //needs del to match pre
                    UngroundPredicate up_new_pre;
                    up_new_pre.predicate_name = pre.predicate_name;
                    for(std::string s : pre.parameter_ids){
                        up_new_pre.parameter_ids.push_back(s);
                    }
                    for(std::string s : pre.parameter_types){
                        up_new_pre.parameter_types.push_back(s);
                    }
                    skills.at(i).del_effects.push_back(up_new_pre);
                }
            }
        }

        //Add abstract versions of each add and del effect
        if(STAMINA){
            bool param_added = false;//only add new abstract_... parameter once.
            std::string relevant_param;
            std:;string relevant_type;
            for(unsigned j = 0; j < s.parameter_types.size(); ++j){
                if(s.parameter_types.at(j) == hasAbstractTypes){
                    param_added = true;
                    relevant_param = s.parameter_ids.at(j);
                    relevant_type = s.parameter_types.at(j);
                    skills.at(i).parameter_ids.push_back("abstract" + skills.at(i).parameter_ids.at(j));
                    skills.at(i).parameter_types.push_back(relevant_type);
                    skills.at(i).param_in_skiros.push_back(false);
                }
            }
            if(param_added){
                //If contained type with abstract instances (in this case Manipulatable only)
                //then need to add isOfType precondition and duplicate add and del effects
                UngroundPredicate up;
                up.predicate_name = "isOfType";
                up.parameter_ids.push_back(relevant_param);
                up.parameter_types.push_back(relevant_type);
                up.parameter_ids.push_back("abstract" + relevant_param);
                up.parameter_types.push_back(relevant_param);
                skills.at(i).preconditions.push_back(up);

                for(UngroundPredicate up_del : s.del_effects){
                    for(std::string p : up_del.parameter_ids){
                        if(p == relevant_param){
                            UngroundPredicate up_new;
                            up_new.predicate_name = up_del.predicate_name;
                            for(std::string s : up_del.parameter_ids){
                                if(s == relevant_param)
                                    up_new.parameter_ids.push_back("abstract" + s);
                                else
                                    up_new.parameter_ids.push_back(s);
                            }
                            skills.at(i).del_effects.push_back(up_new);                  
                        }
                    }
                }
                for(UngroundPredicate up_add : s.add_effects){
                    for(std::string p : up_add.parameter_ids){
                        if(p == relevant_param){
                            UngroundPredicate up_new;
                            up_new.predicate_name = up_add.predicate_name;
                            for(std::string s : up_add.parameter_ids){
                                if(s == relevant_param)
                                    up_new.parameter_ids.push_back("abstract" + s);
                                else
                                    up_new.parameter_ids.push_back(s);
                            }
                            skills.at(i).add_effects.push_back(up_new);                     
                        }
                    }
                }
            }
        }
    }

    //For abstract types.
    if(STAMINA){
        planner_model.addPredicate("isOfType", {hasAbstractTypes, hasAbstractTypes}, false);
    }
}

void PlannerInterface::initProblem()
{
    if(getWorldHandle()==NULL) return;

    ROS_INFO("Getting Objects");
    planner_model.findWorldObjects(); //get all objects of each type
}


void PlannerInterface::setGoal(std::vector<skiros_wm::Element> conditions, std::vector<std::string> pddl_goals)
{
    planner_model.setGoal(conditions, pddl_goals);
}

void PlannerInterface::outputPDDL(){
    printDomainFile();
    printProblemFile();
}

void PlannerInterface::parseSkill(Element skill)
{
    //Do not add skills more than once
    for(PM_Skill s : skills){
        if(s.skill_name == skill.label()){
            ROS_INFO_STREAM("PARSED SKILL " << skill.label() << " TWICE, IGNORING");
            return;
        }
    }

    PM_Skill s;
    ROS_INFO_STREAM("Parsing Skill: " << skill.label());
    s.skill_name = skill.label();
    
    //Set STAMINA specific properties
    if(STAMINA){
        std::size_t found = s.skill_name.find(requireSequenceNumbers);
        if(found != std::string::npos){
            s.measure_distance = true;
        }
        found = s.skill_name.find(requirePartNumbers);
        if(found != std::string::npos){
            s.require_parts = true;
        }
    }

    for(auto pair : skill.properties()){
        skiros_common::Param p = pair.second;
        if(p.specType()!=skiros_common::symbolic && p.type()==typeid(Element)){ //Symbolic are parameters like description, version (not interested in those)            
            std::string type = p.getValue<Element>().type();
            planner_model.addType(type);
            s.param_in_skiros.push_back(true);
            s.parameter_types.push_back(type);
            s.parameter_ids.push_back(p.key());
        }
    }

    std::vector<Element> pre_conditions = getPreConditions(skill);
    for(Element e : pre_conditions)
        s.preconditions.push_back(parseUngroundPredicate(e));

    std::vector<Element> post_conditions = getPostConditions(skill);
    for(Element e : post_conditions){
        if(e.properties("hasDesiredState").getValue<bool>())
            s.add_effects.push_back(parseUngroundPredicate(e));
        else
            s.del_effects.push_back(parseUngroundPredicate(e));
    }

    skills.push_back(s);
}
    
UngroundPredicate PlannerInterface::parseUngroundPredicate(Element e)
{
    UngroundPredicate up;
    up.predicate_name = addPredicate(e); //Adds predicate to planning model if doesn't exist
    //Element will have a subject and subjectType but not always an object/objectType
    up.parameter_ids.push_back(e.properties("hasSubject").getValue<string>());
    up.parameter_types.push_back(e.properties("hasSubjectType").getValue<string>());
    if(e.properties("hasObject").getValue<string>() != ""){
        up.parameter_ids.push_back(e.properties("hasObject").getValue<string>());
        up.parameter_types.push_back(e.properties("hasObjectType").getValue<string>());
    }
    return up;    
}




std::string PlannerInterface::addPredicate(Element e){
    //Only add if doesn't already exist
    if(planner_model.hasPredicate(e.type()))
        return e.type();
    std::vector<std::string> args;
    args.push_back(e.properties("hasSubjectType").getValue<string>());
    if(e.properties("hasObject").getValue<string>() != ""){
        args.push_back(e.properties("hasObjectType").getValue<string>());
    }
    planner_model.addPredicate(e.type(), args, true);
    return e.type();
}

std::vector<Element> PlannerInterface::getRobotSkills(Element robot)
{
    return getWorldHandle()->getChildElements(robot, relation::Str[relation::hasSkill]);
}

std::vector<Element> PlannerInterface::getRobots()
{
    return getWorldHandle()->resolveElement(Element(concept::Str[concept::Agent]));
}

std::vector<Element> PlannerInterface::getPreConditions(Element skill)
{
    return getWorldHandle()->getChildElements(skill, relation::Str[relation::hasPreCondition]);
}

std::vector<Element> PlannerInterface::getPostConditions(Element skill)
{
    return getWorldHandle()->getChildElements(skill, relation::Str[relation::hasPostCondition]);
}

void PlannerInterface::printDomainFile(){
    ROS_INFO_STREAM("Writing domain to domain.pddl...");
    std::ofstream domainFile;
    domainFile.open (ros::package::getPath("skiros_task_planner_plugin")+"/domain.pddl");
    domainFile << "(define (domain stamina)\n";
    if(STAMINA)
        domainFile << "(:requirements :typing :fluents)\n";
    else
        domainFile << "(:requirements :typing)\n";
    domainFile << planner_model.getTypesPDDL();
    domainFile << planner_model.getPredicatesPDDL();
    if(STAMINA){
        domainFile << "(:functions\n";
        domainFile << "\t(distance ?x - Location ?y - Location)\n";
        domainFile << "\t(parts ?x - Location)\n";
        domainFile << ")\n";
    }
    for(PM_Skill s : skills){
        domainFile << s.to_durative_pddl();
    }
    domainFile << ")\n";
    domainFile.close();
    ROS_INFO_STREAM("\t...done!");
}

void PlannerInterface::printProblemFile(){
    ROS_INFO_STREAM("Writing problem file to p01.pddl...");
    std:ofstream problemFile;
    problemFile.open (ros::package::getPath("skiros_task_planner_plugin")+"/p01.pddl");
    problemFile << "(define (problem stamina1) (:domain stamina)\n";
    problemFile << planner_model.getObjectsPDDL();
    problemFile << planner_model.getInitialStatePDDL();
    problemFile << planner_model.getGoalPDDL();
    ROS_INFO_STREAM("GOAL: " << planner_model.getGoalPDDL());
//    problemFile << "(:metric minimize (total-cost) )\n";  
    problemFile << ")\n";
    problemFile.close();
    ROS_INFO_STREAM("\t...done!");
}

inline bool file_exists (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

std::vector<Element> PlannerInterface::callPlanner()
{
    plan.clear();
    std::vector<Element> return_vec;
    
    bool use_modified_TFD = true;
    if(!use_modified_TFD)
        ROS_INFO("EDIT PlannerInterface::callPlanner() to add call to external planner");
    /*
    To add your own external planner the following must be implemented:
    Remove any old output from previous calls to planner
    Call Planner
    Read output and convert back to skills format
    Return plan if found
    Cleanup planner output if needed
    
    see below for example using slightly modified version of TFD.
    */

    if(use_modified_TFD){
        //Remove any old output from previous calls to planner
        int i = 1;
        while(true){
            std::stringstream ss;   
            ss << "out." << i;
            std::string s = ss.str();
            if(std::remove(s.c_str()) != 0){
                break;
            }
            ++i;
        }

        //Call Planner
        string s = "plan.py \"y+Y+a+T+10+t+5+e+r+O+1+C+1\" " + ros::package::getPath("skiros_task_planner_plugin") + "/domain.pddl " + ros::package::getPath("skiros_task_planner_plugin") + "/p01.pddl out";
        
        //Read output and convert back to skills format
        system(s.c_str());
        std::ifstream planFile;
        i = 1;
        while(true){
            std::stringstream ss;
            ss << "out." << i;
            if(!file_exists(ss.str())){
                --i;
                break;
            }
            ++i;
        }
        ROS_INFO_STREAM("Using plan " << i);
        std::stringstream pfile;
        pfile << "out." << i;
        planFile.open(pfile.str().c_str());
        std::string line;
        while(std::getline(planFile, line))
        {
            if(line.at(0) != ';'){
                std::vector<std::string> elems = split(line, ' ');
                GroundSkill gs;
                std::string s_name = elems.at(1);
                s_name.erase(std::remove(s_name.begin(), s_name.end(), '('), s_name.end());
                gs.skill_name = s_name;
                for(unsigned i = 2; i < elems.size()    -1; ++i){
                    std::string o_name = elems.at(i);
                    o_name.erase(std::remove(o_name.begin(), o_name.end(), ')'), o_name.end());
                    gs.obj_names.push_back(o_name);
                }
                plan.push_back(gs);
            }
        }

        //Dump plan
        for(GroundSkill gs : plan){
            ROS_INFO_STREAM(gs.to_str());
        }

        //Return plan if found
        if(plan.size() > 0){
            std::cout << "Plan Found!" << std::endl;
            for(GroundSkill gs : plan)
            {
                gs.skill_name.erase(std::remove(gs.skill_name.begin(), gs.skill_name.end(), '('), gs.skill_name.end());
                Element e_skill = e_skills_.at(gs.skill_name);
                PM_Skill p_skill;
                for(PM_Skill s : skills)
                {
                    if(s.skill_name==gs.skill_name)
                    {
                        //FINFO(s.skill_name);
                        p_skill = s;
                        break;
                    }
                }
                for(int i=0; i<gs.obj_names.size();i++)
                {
                    if(p_skill.param_in_skiros.at(i)){
                        std::string param_id = p_skill.parameter_ids[i];
                        std::string obj_name = gs.obj_names[i];
                        //obj_name.erase(std::remove(obj_name.begin(), obj_name.end(), ')'), obj_name.end());
                        //FINFO(obj_name);
                        //FINFO(param_id);
                        Element object;
                        if(getWorldHandle()->isElementUri(obj_name)) object = getWorldHandle()->getElementFromUri(obj_name);
                        else object = getWorldHandle()->getDefaultElement(obj_name);
                        e_skill.properties(param_id).setValue(object);
                    }
                }
                return_vec.push_back(e_skill);
            }
        }
        else
            std::cout << "No Plan Found!" << std::endl;
    }

    return return_vec;

}

void PlannerInterface::dumpAll(){
    std::cout << planner_model.getTypesPDDL() << std::endl;
    std::cout << planner_model.getPredicatesPDDL() << std::endl;
    if(skills.size() == 0)
        std::cout << "Skills List: Empty" << std::endl;
    else{
        for(PM_Skill s : skills){
            std::cout << s.to_durative_pddl() << std::endl;
        }
    }
    std::cout << planner_model.getObjectsPDDL() << std::endl;
    std::cout << planner_model.getInitialStatePDDL() << std::endl;
    std::cout << planner_model.getGoalPDDL() << std::endl;
}


/*helper functions*/
std::vector<std::string> &PlannerInterface::split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> PlannerInterface::split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}


bool PlannerInterface::isSpatialRelation(UngroundPredicate up){
    if(up.predicate_name == "RobotAtLocation" || up.predicate_name == "Holding" || up.predicate_name == "ObjectAtLocation" || up.predicate_name == "Carrying")
        return true;
    return false;
}

bool PlannerInterface::isRelatedSpatialRelation(UngroundPredicate up1, UngroundPredicate up2){
    //Returns true if up1 is a related spatial relation to up2
    //i.e if up1 is also a spatial relation and shares an argument
    if(!isSpatialRelation(up1))
        return false;
    for(std::string param1 : up1.parameter_ids){
        for(std::string param2 : up2.parameter_ids){
            if(param1 == param2)
                return true;
        }
    }
    return false;
}

}
}
