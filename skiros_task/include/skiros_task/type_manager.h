#ifndef TYPE_MANAGER
#define TYPE_MANAGER

#include <ros/package.h>
#include <skiros_world_model/world_model_interface.h>

namespace skiros_wm
{
namespace pddl
{

//A type is defined by its name and its supertype
//The type 'object' is reserved for the global supertype.
struct PDDL_Type{
    std::string type_name;
    PDDL_Type * supertype;
    unsigned depth; //Number of supertypes above it in heirarchy
    std::vector<std::string> objects_of_type;
};

struct PDDL_function_value{
    std::string name;
    std::vector<std::string> args;
    int value;

    std::string to_pddl(){
        std::stringstream ss;
        ss << "(= (" << name << " ";
        for(std::string a : args){
            ss << a << " ";
        }
        ss << ") " << value << ")";
        return ss.str();
    }
};

struct PDDL_Predicate{
	std::string predicate_name;
	std::vector<std::string> predicate_args;
	bool in_skiros = true;
	std::string to_pddl(){
        std::string s;
        s += "(" + predicate_name;
        for(unsigned i = 0; i < predicate_args.size(); ++i){
            s += " ?" + predicate_args.at(i).substr(0,1) + " - " + predicate_args.at(i);
        }
        s += ")";
        return s;
    }
};

struct GroundPredicate{
    std::string predicate_name;
    std::vector<std::string> predicate_obj_names;

    GroundPredicate(){}
    GroundPredicate(std::string pddl_predicate)
    {
        std::stringstream ss(pddl_predicate);
        std::string s;
        getline(ss, s, '(');
        predicate_name = s;
        while(getline(ss, s, ' ')){
            s.erase(std::remove(s.begin(), s.end(), ','), s.end());
            s.erase(std::remove(s.begin(), s.end(), ')'), s.end());
            predicate_obj_names.push_back(s);
        }
    }

    std::string to_pddl(){
        std::string s;
        s += "(" + predicate_name + " ";
        for(std::string o : predicate_obj_names){
            s += o + " ";
        }
        s += ")";
        return s;
    }
};

struct HiddenPropertyElement{
	Element element; //Element types that have the property
	std::string associatedPredicate; //Predicate name that the property encodes
	const char * data_property; //property name to search for
	std::string type_name; //type of object hidden in the property.
};


class Type_Manager{
    std::vector<std::string> const relevant_types = {
        "LargeBox", "SmallBox", "Rack", "Location",
        "Agent", "Arm", "Gripper", 
        "Manipulatable", "Kit", "Cell"
    };
    PDDL_Type global_super_type;
    unsigned max_depth;
    boost::shared_ptr<WorldModelInterface> wm_;
    std::vector<PDDL_Type> types;
    std::vector<PDDL_Predicate> predicates;
    std::vector<GroundPredicate> initial_state;
    std::vector<GroundPredicate> goal;
    std::vector<PDDL_function_value> function_values;

    std::vector<std::pair<std::string, std::string> > part_instances;

    bool containsNoUsefulParts(Element e);
	void addHiddenObjects(HiddenPropertyElement hpe);
	void addObjectByType(std::string type_name, std::string object_name);
    Element getRack(Element e, int depth);

	std::vector<HiddenPropertyElement> hidden_elements;

public:
    Type_Manager();
    ~Type_Manager(){};

    boost::shared_ptr<WorldModelInterface> getWorldHandle(){return wm_;}
    //SkillsElementMap getSkillsMap(){return e_skills_;}
    void initWorldInterface(boost::shared_ptr<WorldModelInterface> wm){wm_ = wm;}


    void reset();

	void setSuperType(PDDL_Type type_to_set, std::string current_type);
	void findWorldObjects();
	void setGoal(std::vector<skiros_wm::Element> conditions, std::vector<std::string> pddl_goals);


	//Accessor
    bool addType(std::string t);
    void addPredicate(std::string p_name, std::vector<std::string> p_args, bool in_skiros);
    PDDL_Type getType(std::string t);
    std::vector<std::string> getAllObjects(std::string type);
    void setTypeHeirarchy();
    bool hasPredicate(std::string pred_name);
    void addGroundPredicate(std::string name, std::vector<std::string> args);


    //Parsing
    GroundPredicate parseGroundPredicate(Element e);

    //PDDL methods
    std::string getTypesPDDL();
    std::string getPredicatesPDDL();
    std::string getObjectsPDDL();
    std::string getInitialStatePDDL();
    std::string getGoalPDDL();

};

}
}
#endif // TYPE_MANAGER
