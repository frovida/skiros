###  Skill based framework for ROS (SkiROS) from the RVMI lab, Aalborg University Copenhagen, Denmark

[www.rvmi.aau.dk(RVMI webpage)](http://homes.m-tech.aau.dk/mrp/skiros/)

Last update: 25/05/2016  

**Compatibility**: Has been tested with Ubuntu 14.04 and ROS Indigo.

**SkiROS** is a collection of ROS packages to develop, test and deploy behaviours for autonomous robots. Using SkiROS, the developer can split complex *tasks* into *skills*, that get composed automatically at run-time to solve goal oriented missions. The *skills* can themselves be divided into an arbitrary amount of submodules, called *primitives*. Moreover, the framework helps to manage the robot knowledge with the support of a shared semantic world model. 
Using *discrete reasoners*, it is possible to embed in the code reasoning routines with an high level of abstraction. 

The development process consist of two steps: specify the domain knowledge in a OWL ontology and develop the plug-ins. 

The ontology, coded in one or more .owl file, defines which data, concepts, relations and individuals are possible to store to and retrieve from the world model. The knowledge base can be extended from the developer at will. It is possible to modify the default OWL loading path (skiros/owl), by specifying the parameter ”skiros/owl workspace”. All the OWL files found in the specified path are loaded from the system at boot. To create and edit ontologies, we suggest to use the GUI [**Protege_5.0**](http://protege.stanford.edu/download/protege/5.0/snapshots/).

It is possible to generate an header file from the ontology, using the command:

* rosrun skiros_world_model uri_header_generator

This create a default_uri.h header into skiros_config package. This header can be used into your code to simplify the coding process and avoid mispelling.

All the system core functionalities are imported from the extern as plug-ins, using the standard ROS plugin system. The plug-ins are the components that implement the real functionalities and allow the developer to tailor the system to his specific needs. The plug-ins are:
* **skill:** an action with pre and post conditions that can be concatenated to form a complete task
* **primitive:** a simple action without pre- and post- conditions, that is concatenated manually from a expert programmer inside a skill and support hierachical composition
* **condition:** a desired world state. It is expressed as a boolean variable (true/false) applied on a property of an element (property condition) or a relation between two elements (relation condition). The plug-in can wrap methods to evaluate the condition using sensors. 
* **discrete reasoner:** an helper class necessary to link the semantic object definition to discrete data necessary for the robot operation
* **task planner:** a plug-in to plan the sequence of skills given a goal state. Any planner compatible with PDDL standard can be used.

### Included packages 

* **skiros:** holds launch files and handful script to install SkiROS dependencies. It is also the standard workspace directory to load ontology files and scenes. (meta-package)  
* **skiros_ common:** common use classes, Param, ParamHandler, NodeSpawner and other utilities.  
* **skiros_msgs:** all ROS messages used in the system.  
* **skiros_config:** holds definition of URIs (generated from the ontology), ROS topic names and other reconfigurable parameters  
* **skiros_resource [work in progress]:** Under development  
* **skiros_primitive [work in progress]:** Under development  
* **skiros_world_model:** holds the world model node, C++ wrappers for ROS interfaces, utilities to treat ontologies and the base class for conditions and reasoners. It provides two databases:  
   1. the **world ontology**, an abstract description of the world structure (data, concepts, relations and individuals)  
   2. the **world scene**, the instance of the world  
* **skiros_skill:** holds the skill manager node and the base class for the skills and the primitives. The skill layer can run one or more skill managers. Each skill manager represent a robot subsystem. It publish its description (hardware, skill and primitives) on the world model and wait for commands from the task layer. The skill managers manage the skills execution using the world_model to store and get information.  
* **skiros_task:** the higher SkiROS layer, contain the task manager node and the base class for task planner plug-in  
* **skiros_rqt_gui:** QT-based GUI for interacting with the SkiROS system  

### Dependencies

* **Oracle database**  
* [**redland, rasqal, raptor:**](http://download.librdf.org/source/)   

These dependencies are necessary to manage Web Ontology Language (OWL) ontologies.

### Install

To install all dependencies, user can run the script in skiros/scripts/install_dependencies.sh. Dependencies are installed in /usr/local.

The package can then be compiled using catkin.

### Launching the system

It is possible to launch the whole system with the GUI using the launch file in skiros/launch:

* roslaunch skiros skiros_system.launch

It is also possible to run singularly the main SkiROS nodes, these are:  

* rosrun skiros_world_model world_model_node
* rosrun skiros_skill skill_manager_node
* rosrun skiros_task task_manager_node

In particular, some users could be interested in using the world model as a stand-alone package.

### Doxygen documentation

The SkiROS doxygen documentation is available at [this](http://homes.m-tech.aau.dk/mrp/skiros/) link.

