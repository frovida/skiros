#include <ros/ros.h>
#include <ros/package.h>
#include <skiros_task/planner_model.h>
#include <skiros_config/declared_uri.h>
#include <skiros_common/param_handler.h>
#include <skiros_world_model/condition.h>
#include <iostream>
#include <boost/algorithm/string.hpp>    


using namespace std;
using namespace skiros_config::owl;


namespace skiros_wm
{
namespace pddl
{

PlannerModel::PlannerModel(){
	global_super_type = PDDL_Type{/*.type_name*/"object", /*.supertype*/nullptr, /*.depth*/0};
    max_depth = 0;

    //Predicates that have to extracted from properties in the World Model
    hidden_elements.push_back(HiddenPropertyElement{
        /*.element*/Element(concept::Str[concept::LargeBox]),
        /*.associatedPredicate*/"ObjectAtLocation",
        /*.data_property*/data::Str[data::partReference],
        /*.type_name*/"Manipulatable"
    });
    hidden_elements.push_back(HiddenPropertyElement{
        /*.element*/Element(concept::Str[concept::SmallBox]),
        /*.associatedPredicate*/"ObjectAtLocation",
        /*.data_property*/data::Str[data::partReference],
        /*.type_name*/"Manipulatable"
    });
}

void PlannerModel::reset(){
	types.clear();
    predicates.clear();
    initial_state.clear();
    function_values.clear();
    goal.clear();
}

/*
Add a new type if it doesn't already exist.
*/
bool PlannerModel::addType(std::string s){
    if(s == "object"){
        std::cout << "ERROR: cannot use global supertype named object in domain definition" << std::endl;
        return false;
    }
    for(PDDL_Type t : types){
        if(t.type_name == s){
            return false;
        }
    }
    ROS_INFO_STREAM("TaskPlanner: Adding NEW type: " << s);
    PDDL_Type t = { /*.type_name*/ s, /*.supertype*/ &global_super_type, /*.depth*/ 1};
    if(max_depth < 1)
        max_depth = 1;
    types.push_back(t);
    return true;
}

void PlannerModel::setTypeHeirarchy(){
    //First find relevant subtypes
    // for(PDDL_Type current_type : types){
    //     std::set<std::string> subs = getWorldHandle()->getSubClasses(current_type.type_name);
    //     ROS_INFO_STREAM("Subtypes of " << current_type.type_name);
    //     for(std::string s: subs){
    //         ROS_INFO_STREAM("\t" << s);
    //     }
    // }
    //addType("SmallBox");
    //addType("LargeBox");

    //Set the supertypes
    for(PDDL_Type t : types){
        setSuperType(t, t.type_name);
    }
}

PDDL_Type PlannerModel::getType(std::string s){
    PDDL_Type ret;
    for(PDDL_Type t : types){
        if(t.type_name == s){
            ret = t;
        }
    }
//    std::cout << "DID NOT FIND TYPE!!" << std::endl;
    return ret;
}

void PlannerModel::setSuperType(PDDL_Type type_to_set, std::string current_type){
    std::set<std::string> super = getWorldHandle()->getSuperClasses(current_type);
    //Stop if reached top of heirarchy
    if(super.size() == 0){
        return;
    }
    for(std::string s : super){
        for(PDDL_Type potential_s : types){
            if(potential_s.type_name == s){
                std::cout << "Set " << s << " supertype of " << type_to_set.type_name << std::endl;
                type_to_set.supertype = &potential_s;
                return;
            }
        }
        //If no match then recurse
        setSuperType(type_to_set, s);
    }          
}

Element PlannerModel::getRack(Element e, int depth){
    if(e.type() == "Rack" || depth >= 5)
        return e;
    Element p = getWorldHandle()->getParentElement(e.id(), "");
    int d = depth + 1;
    return getRack(p, d);
}

// bool PlannerModel::containsNoUsefulParts(Element e){
//     if(!(e.type() == "LargeBox" || e.type() == "SmallBox"))
//         return false;
    
//     std::stringstream goalss;
//     for(GroundPredicate g : goal){
//         goalss <<  g.to_pddl();
//     }
//     ROS_INFO_STREAM(goalss.str());

//     bool inGoal = false;
//     std::vector<std::string> objects = e.properties(data::Str[data::partReference]).getValues<std::string>();
//     for(std::string s : objects){
//         ROS_INFO_STREAM("\t" << s);
//         std::size_t found = goalss.str().find(s);
//         if(found != std::string::npos){
//             return false;
//         }
//     }
//     ROS_INFO_STREAM("Ignoring "  << e.toUrl());
//     return true;
// }

void PlannerModel::findWorldObjects(){

    //For Sequence Numbers
    std::vector<std::pair<std::string, int>>  obj_seq_pair;

    //For each type query WM for all elements of that type and add them to that type list
    for(unsigned i = 0; i < types.size(); ++i)
    {
        ROS_INFO_STREAM("Searching for all " << types.at(i).type_name);
        std::vector<Element> objects_e = getWorldHandle()->resolveElement(Element(types.at(i).type_name));
        for(Element e : objects_e)
        {
            ROS_INFO_STREAM("\t" << e.toUrl() << " " << e.type());
            if(types.at(i).type_name == "Location"){
                //if(containsNoUsefulParts(e)){//i.e. is a largebox or smallbox with part that is not in goal in it
                //    ROS_INFO_STREAM("Ignored Part!");
               // }
                if(e.type() == "Location" || e.type() == "LargeBox" || e.type() == "SmallBox" || e.type() == "ParkingStation"){
                    types.at(i).objects_of_type.push_back(e.toUrl());
                    //try to get sequencenr if exists
                    if(e.hasProperty(data::Str[data::sequenceNr])){
                        std::vector<std::string> objects = e.properties(data::Str[data::sequenceNr]).getValues<std::string>();
                        for(std::string s : objects){
                            std::cout << "Found sequence number " << s << " for " << e.toUrl() << std::endl;
                            obj_seq_pair.push_back(std::make_pair(e.toUrl(), std::stoi(s)+1));
                        }
                    }
                    else if(e.type() == "SmallBox"){//Get the related rack's sequenceNr
                        Element r = getRack(e, 0);
                        if(r.type() == "Rack"){
                            std::vector<std::string> objects = r.properties(data::Str[data::sequenceNr]).getValues<std::string>();
                            for(std::string s : objects){
                                std::cout << "Found sequence number " << s << " for " << e.toUrl() << std::endl;
                                obj_seq_pair.push_back(std::make_pair(e.toUrl(), std::stoi(s)+1));
                            }
                        }
                        else{
                            ROS_INFO_STREAM("WARNING: Could not sequenceNr for SmallBox" << e.id() << "not defined directly and no associated rack found.");
                        }
                    }
                    else{//use default 0
                        ROS_INFO_STREAM("WARNING " << e.type() << "-" << e.id() << " does not have a sequenceNr (using 0 for default)");
                        obj_seq_pair.push_back(std::make_pair(e.toUrl(), 0));
                    }     
                }
            }
            else if(types.at(i).type_name == "Cell"){
                if(e.type() != "RackCell" && e.type() != "LargeBoxCell"){
                    types.at(i).objects_of_type.push_back(e.toUrl());
                }
            }
            else if(types.at(i).type_name == "Manipulatable"){
                types.at(i).objects_of_type.push_back(e.toUrl());
                std::string abs_type = e.type();
                boost::algorithm::to_lower(abs_type);
                part_instances.push_back(std::make_pair(abs_type, e.toUrl()));
                GroundPredicate gp;
                gp.predicate_name = "isOfType";
                gp.predicate_obj_names.push_back(e.toUrl());
                gp.predicate_obj_names.push_back(abs_type);
                initial_state.push_back(gp); 
            }
            else{
                types.at(i).objects_of_type.push_back(e.toUrl());
            }
        }
    }

    //Add the Relations encoded in element properties.
    for(HiddenPropertyElement hpe : hidden_elements){
        addHiddenObjects(hpe);
    }

    //Find the ground predicates that are true in current state
    boost::shared_ptr<skiros_common::ParamHandler> ph(new skiros_common::ParamHandler());
    ph->addParamWithDefaultValue("Subject", Element(""));
    ph->addParamWithDefaultValue("Object", Element(""));

    for(PDDL_Predicate p : predicates){
        //ROS_INFO_STREAM("Querying " << p.predicate_name);
        if(p.in_skiros && p.predicate_name != "FitsIn"){ //FitsIn is coded as partReference of a Cell...
            std:string subject_type = p.predicate_args.at(0);
            std::string object_type = "";
            if(p.predicate_args.size() == 2)
                object_type = p.predicate_args.at(1);

            //ROS_INFO_STREAM("Checking World Model for predicate: " << p.predicate_name);
            boost::shared_ptr<skiros::condition::ConditionBase> c = skiros::condition::loadCondition(getWorldHandle(), ph, p.predicate_name, true, "Subject", "Object");
            
            //Just tries every possible object to see if it's there
            //get all objects(string) of type subject_type
            std::vector<std::string> subject_objects = getAllObjects(subject_type);
            for(std::string subject_object : subject_objects){
                ph->specify("Subject", getWorldHandle()->getElementFromUri(subject_object));
                if(object_type == ""){
                    //std::cout << "trying (" << p.predicate_name << " " << subject_object << ")" << std::endl;
                    if(c->evaluate()){
                        GroundPredicate gp;
                        gp.predicate_name = p.predicate_name;
                        gp.predicate_obj_names.push_back(subject_object);
                        initial_state.push_back(gp);
                    }
                }
                else{
                    std::vector<std::string> object_objects = getAllObjects(object_type);
                    for(std::string object_object : object_objects){
                        ph->specify("Object", getWorldHandle()->getElementFromUri(object_object));
                        //std::cout << "trying (" << p.predicate_name << " " << subject_object << " " << object_object << ")" << std::endl;
                        if(c->evaluate()){
                            //If it's there then add it.
                            GroundPredicate gp;
                            gp.predicate_name = p.predicate_name;
                            gp.predicate_obj_names.push_back(subject_object);
                            gp.predicate_obj_names.push_back(object_object);
                            initial_state.push_back(gp);
                        }
                    }
                }
            }
        }
    }

    // //Add which cell each part can go in;
    std::vector<Element> cells = getWorldHandle()->resolveElement(Element(concept::Str[concept::Cell]));
    std::vector<std::string> world_objects = getAllObjects("Manipulatable");

    boost::shared_ptr<skiros::condition::ConditionBase> fitsin = skiros::condition::loadCondition(getWorldHandle(), ph, "FitsIn", true, "Subject", "Object");
    for(unsigned i = 0; i < cells.size(); ++i)
    {
        for(std::string object_type : world_objects)
        {
            Element cell = cells.at(i);
            Element object = getWorldHandle()->getDefaultElement(object_type);
            ph->specify("Subject", cell);
            ph->specify("Object", object);
            if(fitsin->evaluate())
            {
                //add predicate (FitsIn ?cell ?part)
                GroundPredicate gp;
                gp.predicate_name = "FitsIn";
                gp.predicate_obj_names.push_back(cell.toUrl());
                gp.predicate_obj_names.push_back(object_type);
                initial_state.push_back(gp);

                for(std::pair<std::string, std::string> p_i : part_instances){
                    if(object_type == p_i.first){
                        GroundPredicate gp2;
                        gp2.predicate_name = "FitsIn";
                        gp2.predicate_obj_names.push_back(cell.toUrl());
                        gp2.predicate_obj_names.push_back(p_i.second);
                        initial_state.push_back(gp2);
                    }
                }
            }
        }
    }

    //**************************************************
    //             Add sequence numbers
    //**************************************************
    //sequence numbers are properties of Racks (which contain the smallboxes) and LargeBoxes
    //can associate the largebox numbers directly with the largebox
    //need to work out which rack each smallbox is on

    //Calculate distances (|sequencnr - sequencenr| + 1)
    for(int i = 0; i < obj_seq_pair.size(); ++i){
        for(int j = 0; j < obj_seq_pair.size(); ++j){
            PDDL_function_value f;
            f.name = "distance";
            std::vector<std::string> args;
            args.push_back(obj_seq_pair.at(i).first);
            args.push_back(obj_seq_pair.at(j).first);
            f.args = args;
            f.value = abs(obj_seq_pair.at(i).second - obj_seq_pair.at(j).second) + 1;
            function_values.push_back(f);
        }
    }


}

std::vector<std::string> PlannerModel::getAllObjects(std::string type){
    std::vector<std::string> ret;
    for(PDDL_Type t : types){
        if(t.type_name == type){
            return t.objects_of_type;
        }
    }
    return ret;
}

void PlannerModel::addPredicate(std::string p_name, std::vector<std::string> p_args, bool in_skiros){
    PDDL_Predicate p;
    p.predicate_name = p_name;
    p.predicate_args = p_args;
    p.in_skiros = in_skiros;
    predicates.push_back(p);
}

bool PlannerModel::hasPredicate(std::string pred_name){
    for(PDDL_Predicate p : predicates){
        if(p.predicate_name == pred_name)
            return true;
    }
    return false;
}

std::string PlannerModel::getTypesPDDL(){
    std::stringstream ss;
    ss << "(:types\n\t";
    //top level (object)
    for(unsigned i = 1; i <= max_depth; ++i){
        ss << "\t";
        for(PDDL_Type t : types){
            if(t.depth == i){
                ss << t.type_name << " ";
            }
            if(i > 1){
                ss << " - " << t.supertype->type_name << "\n";
            }
        }
        if(i == 1)
            ss << "- object\n";
    }
    ss << ")\n";
    return ss.str();
}

std::string PlannerModel::getObjectsPDDL(){
    std::stringstream ss;
    ss << "(:objects\n";
    for(PDDL_Type t : types){
        if(t.objects_of_type.size() > 0){
            ss << "\t";
            for(std::string obj : t.objects_of_type){
                ss << obj << " ";
            }
            ss << "- " << t.type_name << "\n";
        }
        else{
            ROS_INFO_STREAM("No objects found of type " << t.type_name);
        }
    }
    ss << ")\n";
    return ss.str();
}

std::string PlannerModel::getPredicatesPDDL(){
    std::stringstream ss;
    ss << "(:predicates\n";
    for(PDDL_Predicate p : predicates){
        ss << "\t" << p.to_pddl() << "\n";
    }
    ss << ")\n";
    return ss.str();
}

std::string PlannerModel::getInitialStatePDDL(){
    std::stringstream ss;
    ss << "(:init\n";
    for(GroundPredicate gp : initial_state){
        ss << "\t" << gp.to_pddl() << "\n";
    }
    for(PDDL_function_value f : function_values){
        ss << "\t" << f.to_pddl() << "\n";
    }
//  ss << "\t(= (total-cost) 0)\n";
    ss << ")\n";
    return ss.str();
}

std::string PlannerModel::getGoalPDDL(){
    std::stringstream ss;
    ss << "(:goal (and\n";
    for(GroundPredicate gp : goal){
        ss << "\t" << gp.to_pddl() << "\n";
    }
    ss << "))\n";
    return ss.str();
}

GroundPredicate PlannerModel::parseGroundPredicate(Element e){
    GroundPredicate gp;
    gp.predicate_name = e.type();
    gp.predicate_obj_names.push_back(e.properties("hasSubject").getValue<string>());
    if(e.properties("hasObject").getValue<string>() != ""){
        gp.predicate_obj_names.push_back(e.properties("hasObject").getValue<string>());
    }
    return gp;    
}

void PlannerModel::addObjectByType(std::string type_name, std::string object_name){
    for(unsigned i = 0; i < types.size(); ++i){
        if(types.at(i).type_name == type_name){
            if(find(types.at(i).objects_of_type.begin(), types.at(i).objects_of_type.end(), object_name) == types.at(i).objects_of_type.end()){
                ROS_INFO_STREAM("Adding object " << object_name << " of type " << type_name);
                types.at(i).objects_of_type.push_back(object_name);
                if(type_name == "Manipulatable"){
                    GroundPredicate gp;
                    gp.predicate_name = "isOfType";
                    gp.predicate_obj_names.push_back(object_name);
                    gp.predicate_obj_names.push_back(object_name);
                    initial_state.push_back(gp); 
                }        
            }
        }
    }
}

void PlannerModel::addGroundPredicate(std::string name, std::vector<std::string> args){
    GroundPredicate gp;
    gp.predicate_name = name;
    gp.predicate_obj_names = args;
    initial_state.push_back(gp);
}

void PlannerModel::setGoal(std::vector<skiros_wm::Element> conditions, std::vector<std::string> pddl_goals){
    for(Element e : conditions)
        goal.push_back(parseGroundPredicate(e));
    for(auto s : pddl_goals)
        goal.push_back(GroundPredicate(s));
}

void PlannerModel::addHiddenObjects(HiddenPropertyElement hpe){
    std::vector<Element> locs = getWorldHandle()->resolveElement(hpe.element);

    //addType(hpe.type_name);
    for(unsigned j = 0; j < locs.size(); ++j){
        Element e = locs.at(j);
        // if(!containsNoUsefulParts(e)){
            std::vector<std::string> objects = e.properties(hpe.data_property).getValues<std::string>();
            for(std::string s : objects){
                if(s != ""){
                    ROS_INFO_STREAM("Adding " << s << " " << hpe.associatedPredicate << " " << e.type());
                    addObjectByType(hpe.type_name, s);
                    GroundPredicate gp;
                    gp.predicate_name = hpe.associatedPredicate;
                    gp.predicate_obj_names.push_back(e.toUrl());
                    gp.predicate_obj_names.push_back(s);
                    initial_state.push_back(gp);
                }
            }
            std::vector<std::string> partNo = e.properties(data::Str[data::nrParts]).getValues<std::string>();
            for(std::string s : partNo){
                PDDL_function_value f;
                f.name = "parts";
                std::vector<std::string> a;
                a.push_back(e.toUrl());
                f.args = a;
                f.value = std::stoi(s);
                function_values.push_back(f);
            }
        // }
    }
}

}
}
