#ifndef PLANNERINTERFACE
#define PLANNERINTERFACE

#include <skiros_task/planner_model.h>
#include <skiros_world_model/world_model_interface.h>

namespace skiros_wm
{
namespace pddl
{

/*
Unground predicate e.g. (at ?p ?x)
predicate_name = name of predicate e.g. "at"
parameter_ids = ids of parameters e.g. {"?p", "?x"}
parameter_types = types of parameters e.g. {robot, location}

parameter_ids only required to match skill definitions could be automatically generated otherwise
*/
struct UngroundPredicate{
    std::string predicate_name;
    std::vector<std::string> parameter_ids;
    std::vector<std::string> parameter_types;
    std::string to_pddl(){ //Output (name ?a ?b ?c)
        std::string s;
        s += "(" + predicate_name;
        for(unsigned i = 0; i < parameter_ids.size(); ++i){
            s += " ?" + parameter_ids.at(i);
        }
        s += ")";
        return s;
    }
};

/*
A skill converted into a planning action e.g. Pick.
skill_name = name of skill e.g. "Pick"
parameter_ids = parameters e.g. {"?r, ?p, ?loc"}
*/
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
    /*
    Convert to an action
    */
    std::string to_pddl(){
        std::stringstream s;
        //open
        s << "(:action " << skill_name << "\n";
        //parameters
        s << "\t:parameters (";
        for(unsigned i = 0; i < parameter_ids.size(); ++i){
            s << " ?" << parameter_ids.at(i) << " - ";  
            s << parameter_types.at(i);
        }   
        s << ")\n";
        //precondition
        s << "\t:precondition (and\n";
        for(UngroundPredicate up : preconditions){
            s << "\t\t" << up.to_pddl() << "\n";
        }
        for(UngroundPredicate up : negative_preconditions){
            s << "\t\t(not " << up.to_pddl() << ")\n";
        }
        s << "\t)\n";
        //effect
        s << "\t:effect (and\n";
        //-ve effs
        for(UngroundPredicate up : del_effects){
            s << "\t\t(not " << up.to_pddl() << ")\n";
        }
        //+ve effs
        for(UngroundPredicate up : add_effects){
            s << "\t\t" << up.to_pddl() << "\n";
        }
        //s << "\t\t(increase (total-cost) 1)\n";
        s << "\t)\n";
        //close
        s << ")\n";
        return s.str();
    }
    /*
    Converts to a durative action
    */
    std::string to_durative_pddl(){
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
            s << "\t\t(at start (not " << up.to_pddl() << "))\n";
        }
        //+ve effs
        for(UngroundPredicate up : add_effects){
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

/*
An instantiated action with parameters ground to object names.
*/
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

class PlannerInterface
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
    PlannerModel planner_model;
    std::vector<PM_Skill> skills;
    
    //Return
    std::vector<GroundSkill> plan;

public:
    PlannerInterface(){}
    ~PlannerInterface(){}

    //Skiros functions
    boost::shared_ptr<WorldModelInterface> getWorldHandle(){return wm_;}
    void initWorldInterface(boost::shared_ptr<WorldModelInterface> wm){wm_ = wm; planner_model.initWorldInterface(wm);}

    void dumpAll();
    void resetAll();

    //Main functions
    void initDomain();
    void setGoal(std::vector<skiros_wm::Element> conditions, std::vector<std::string> pddl_goals);
    void initProblem();
    void outputPDDL();
    std::vector<Element> callPlanner();

private:
    PDDL_Type getType(std::string s);
    std::string addPredicate(Element e);
    void parseSkill(Element skill);
    UngroundPredicate parseUngroundPredicate(Element condition);

    void printDomainFile();
    void printProblemFile();

    //Interface with SkiROS
    std::vector<Element> getRobots();
    std::vector<Element> getRobotSkills(Element robot);
    std::vector<Element> getPreConditions(Element skill);
    std::vector<Element> getPostConditions(Element skill);

    //helper methods
    bool isSpatialRelation(UngroundPredicate up);
    bool isRelatedSpatialRelation(UngroundPredicate up1, UngroundPredicate up2);
    std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
    std::vector<std::string> split(const std::string &s, char delim);
};

}
}


#endif // PLANNERINTERFACE
