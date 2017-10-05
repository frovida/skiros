/*
 * Copyright (c) 2011
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <skiros_rqt_gui/skiros_gui.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QLineEdit>
#include <QTimer>
#include <QDir>
#include <fstream>
#include <streambuf>


#include "skiros_world_model/world_model_interface.h"
#include <skiros_world_model/utility.h>
#include "skiros_config/declared_uri.h"
#include <skiros_config/node_names.h>
#include <skiros_msgs/TmTaskExe.h>
#include <skiros_msgs/TmModifyTask.h>
#include <skiros_msgs/WmSceneLoadAndSave.h>
#include <skiros_msgs/TmQueryTask.h>
#include <skiros_common/utility.h>
#include "skiros_msgs/TmGoal.h"
#include "skiros_rqt_gui/custom_dialog_window.h"

using namespace skiros_config::owl;
using namespace skiros_config;
using namespace skiros_wm;
using namespace std;

const std::string skill_exe_author = "skiros_gui";

namespace skiros_gui {


SkirosGui::SkirosGui()
  : rqt_gui_cpp::Plugin()
  , widget_(new QWidget)
{
    int id = qRegisterMetaType<skiros_msgs::ModuleStatus>("ModuleStatusMsg");
    int id1 = qRegisterMetaType<skiros_msgs::TmMonitor>("TmMonitorMsg");
    int id2 = qRegisterMetaType<skiros_wm::Element>("Element");
    setObjectName("SkiROS");
}


//Plugin functions

void SkirosGui::initPlugin(qt_gui_cpp::PluginContext& context)
{
  ///Basic initializations
  nh_ptr_.reset(new ros::NodeHandle(""));
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  wm_ptr_.reset(new skiros_wm::WorldModelInterface(*nh_ptr_, true));
  if(!wm_ptr_->isConnected()) FWARN("[SkirosGui::initPlugin] World model seems down, waiting...");
  if(!wm_ptr_->waitConnection(ros::Duration(3.0)) && ros::ok())
  {
      FWARN("[SkirosGui::initPlugin] World model seems down, waiting...");
  }
  if(!ros::ok())
      return;
  FINFO("[SkirosGui::initPlugin] Connected to world model");

  ros::Duration(0.5).sleep();//Wait half second to avoid a weird bug where I don't detect the presence of robots

  skill_interface_ptr.reset(new skiros_skill::SkillLayerInterface());
  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  ///Task manager interfaces
  task_exe_pub_ = nh_ptr_->advertise<skiros_msgs::TmTaskExe>(std::string(task_mgr_node_name) + task_exe_tpc_name, 10);
  task_monitor_sub_ = nh_ptr_->subscribe(std::string(task_mgr_node_name) + task_monitor_tpc_name, 5, &SkirosGui::tmFeedbackReceived,  this);
  task_modify_ = nh_ptr_->serviceClient<skiros_msgs::TmModifyTask>(std::string(task_mgr_node_name) + task_modify_srv_name);
  task_query_ = nh_ptr_->serviceClient<skiros_msgs::TmQueryTask>(std::string(task_mgr_node_name) + task_query_srv_name);

  ///General
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(refreshTimerCb()));
  timer->start(200);
  ui_.robot_combo_box->setCurrentIndex(ui_.robot_combo_box->findText(""));
  connect(ui_.robot_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(robotIndexChanged(int)));
  connect(ui_.modality_checkBox, SIGNAL(toggled(bool)), this, SLOT(modalityButtonClicked()));
  connect(ui_.exe_task_button, SIGNAL(pressed()), this, SLOT(exeTaskButtonClicked()));
  connect(ui_.stop_task_button, SIGNAL(pressed()), this, SLOT(stopTaskButtonClicked()));
  connect(this, SIGNAL(robotFeedbackReceived(const skiros_msgs::ModuleStatus &)), this, SLOT(robotMonitorCb(const skiros_msgs::ModuleStatus &)));
  connect(this, SIGNAL(tmFeedbackReceived(const skiros_msgs::TmMonitor &)), this, SLOT(taskMonitorCb(const skiros_msgs::TmMonitor &)));

  ///Goal tab
  ui_.goal_table_view->setModel(&goal_model_);
  task_plan_ = nh_ptr_->serviceClient<skiros_msgs::TmGoal>(std::string(task_mgr_node_name) + task_plan_srv_name);
  connect(ui_.condition_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(conditionIndexChanged(int)));
  connect(ui_.condition_refresh_button, SIGNAL(pressed()), this, SLOT(refreshButtonClicked()));
  connect(ui_.condition_add_button, SIGNAL(pressed()), this, SLOT(addConditionButtonClicked()));
  connect(ui_.condition_remove_button, SIGNAL(pressed()), this, SLOT(removeConditionButtonClicked()));
  connect(ui_.plan_button, SIGNAL(pressed()), this, SLOT(taskPlanButtonClicked()));

  ///Task tab
  ui_.skills_table_view->setModel(&skill_model_);
  ui_.skill_combo_box->setCurrentIndex(ui_.skill_combo_box->findText(""));
  connect(ui_.skill_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(skillIndexChanged(int)));
  //ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.add_skill_button, SIGNAL(pressed()), this, SLOT(addSkillButtonClicked()));
  connect(ui_.remove_skill_button, SIGNAL(pressed()), this, SLOT(removeSkillButtonClicked()));
  updateTask();

  ///Modules tab
  ui_.module_combo_box->setCurrentIndex(ui_.module_combo_box->findText(""));
  connect(ui_.module_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(moduleIndexChanged(int)));
  //ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.exe_module_button, SIGNAL(pressed()), this, SLOT(exeModuleButtonClicked()));
  connect(ui_.stop_module_button, SIGNAL(pressed()), this, SLOT(stopModuleButtonClicked()));

  ///World model tab
  wm_model_ptr_ = new TreeModel(wm_ptr_->getElement(0));
  ui_.world_model_tree_view->setModel(wm_model_ptr_);
  ui_.world_model_tree_view->setModel(wm_model_ptr_);
  connect(ui_.world_model_tree_view->selectionModel(), SIGNAL(currentChanged(const QModelIndex &, const QModelIndex &)), this, SLOT(onWmTreeSelectionChanged(const QModelIndex &, const QModelIndex &)));
  connect(ui_.remove_object_button, SIGNAL(pressed()), this, SLOT(removeWmElementButtonClicked()));
  connect(ui_.add_object_button, SIGNAL(pressed()), this, SLOT(addWmElementButtonClicked()));
  connect(ui_.modify_object_button, SIGNAL(pressed()), this, SLOT(modifyWmElementButtonClicked()));
  connect(ui_.saveScene_button, SIGNAL(pressed()), this, SLOT(saveSceneButtonClicked()));
  connect(ui_.loadScene_button, SIGNAL(pressed()), this, SLOT(loadSceneButtonClicked()));
  scene_load_save_ = nh_ptr_->serviceClient<skiros_msgs::WmSceneLoadAndSave>(std::string(world_model_node_name) + wm_scene_load_save_srv_name);
  interactive_m_server_.reset( new interactive_markers::InteractiveMarkerServer("skiros_obj_edit") );
  ///Logging tab
  connect(ui_.startstoplog_button, SIGNAL(pressed()), this, SLOT(startLogButtonClicked()));
  connect(ui_.logFile_lineEdit, SIGNAL(returnPressed()), this, SLOT(logFileChanged()));
}

void SkirosGui::shutdownPlugin()
{
    saveLog();
}

void SkirosGui::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    instance_settings.setValue("sceneName", ui_.sceneFile_lineEdit->text());
    instance_settings.setValue("advancedOptions", ui_.modality_checkBox->isChecked());
    instance_settings.setValue("size", widget_->size());
    instance_settings.setValue("logFile", ui_.logFile_lineEdit->text());
    instance_settings.setValue("saveLog", logging_);
}

void SkirosGui::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    std::string sceneNameParam;
    ros::NodeHandle skiros_nh(skiros_config::skiros_namespace);
    skiros_nh.param<std::string>(skiros_config::scene_name, sceneNameParam, "");
    QString sceneName = instance_settings.value("sceneName", "").toString();
    if(sceneNameParam!="")
        ui_.sceneFile_lineEdit->setText(sceneNameParam.c_str());
    else if(!sceneName.isEmpty())
        ui_.sceneFile_lineEdit->setText(sceneName);
    QSize size = instance_settings.value("sceneName", "").toSize();
    widget_->resize(size);
    auto check = instance_settings.value("advancedOptions", "").toBool();
    ui_.modality_checkBox->setChecked(check);
    QString logFile = instance_settings.value("logFile", "").toString();
    if(logFile!="")
        ui_.logFile_lineEdit->setText(logFile);
    else
    {
        QString directory = QDir::homePath() + "/skiros/log.txt";
        ui_.logFile_lineEdit->setText(directory);
    }
    logFileChanged();
    if(instance_settings.value("saveLog", "").toBool())
        this->startLogButtonClicked();
}

//Goal tab


void SkirosGui::taskPlanButtonClicked()
{
    skiros_msgs::TmGoal msg;
    for(int i=0;i<goal_model_.rowCount(QModelIndex());i++)
    {
        msg.request.conditions.push_back(skiros_wm::element2msg(goal_model_.getRow(i).data()));
    }
    if(task_plan_.call(msg))
    {
    }
}

void SkirosGui::conditionIndexChanged(int index)
{
    ui_.condition_sub_combo_box->clear();
    ui_.condition_obj_combo_box->clear();
    Element condition = qvariant_cast<Element>(ui_.condition_combo_box->itemData(index));
    string sub_type = condition.properties("hasSubjectType").getValue<string>();
    ui_.condition_description->setText(condition.properties("description").getValue<std::string>().c_str());
    string obj_type = condition.properties("hasObjectType").getValue<string>();
    {
        if(condition.properties("allowsAbstractTypes").getValue<bool>())
        {
            std::stringstream ss(wm_ptr_->queryOntology("SELECT DISTINCT ?x WHERE { { ?x rdf:type " + wm_ptr_->addPrefix(sub_type) + ". } UNION " +
                                                        "{?z rdfs:subClassOf "+ wm_ptr_->addPrefix(sub_type) +". ?x rdf:type ?z . } UNION " +
                                                        "{?z rdfs:subClassOf "+ wm_ptr_->addPrefix(sub_type) +". ?v rdfs:subClassOf ?z. ?x rdf:type ?v. } " + "}"));
            std::string temp;
            ss >> temp;
            while(!ss.eof())
            {
                if(temp!="")
                {
                    skiros_wm::Element e;
                    if(temp.find('-')!=std::string::npos)
                        e = wm_ptr_->getElementFromUri(temp);
                    else
                        e = wm_ptr_->getDefaultElement(temp);
                    ui_.condition_sub_combo_box->addItem(e.printState("", false).c_str(), QVariant::fromValue(e));
                }
                ss >> temp;
            }
        }
        else
        {
            std::vector<skiros_wm::Element> v = wm_ptr_->resolveElement(Element(sub_type));
            for(Element e : v)
            {
                ui_.condition_sub_combo_box->addItem(e.printState("", false).c_str(), QVariant::fromValue(e));
            }
        }
    }
    if(obj_type!="" )
    {
        if(condition.properties("allowsAbstractTypes").getValue<bool>())
        {
            std::stringstream ss(wm_ptr_->queryOntology("SELECT DISTINCT ?x WHERE { { ?x rdf:type " + wm_ptr_->addPrefix(sub_type) + ". } UNION " +
                                                        "{?z rdfs:subClassOf "+ wm_ptr_->addPrefix(sub_type) +". ?x rdf:type ?z . } UNION " +
                                                        "{?z rdfs:subClassOf "+ wm_ptr_->addPrefix(sub_type) +". ?v rdfs:subClassOf ?z. ?x rdf:type ?v. } " + "}"));
            std::string temp;
            ss >> temp;
            while(!ss.eof())
            {
                if(temp!="")
                {
                    skiros_wm::Element e;
                    if(temp.find('-')!=std::string::npos)
                        e = wm_ptr_->getElementFromUri(temp);
                    else
                        e = wm_ptr_->getDefaultElement(temp);
                    ui_.condition_obj_combo_box->addItem(e.printState("", false).c_str(), QVariant::fromValue(e));
                }
                ss >> temp;
            }
        }
        else
        {
            std::vector<skiros_wm::Element> v = wm_ptr_->resolveElement(Element(obj_type));
            for(Element e : v)
            {
                ui_.condition_obj_combo_box->addItem(e.printState("", false).c_str(), QVariant::fromValue(e));
            }
        }
    }
}

void SkirosGui::refreshButtonClicked()
{
    std::vector<skiros_wm::Element> v = wm_ptr_->resolveElement(Element(concept::Str[concept::Condition]));
    for(Element e : v)
    {
        if(ui_.condition_combo_box->findText(e.type().c_str(), Qt::MatchCaseSensitive)<0)
        {
            ui_.condition_combo_box->addItem(e.type().c_str(), QVariant::fromValue(e));
        }
    }
}

void SkirosGui::addConditionButtonClicked()
{
    skiros_wm::Element wm_condition = ui_.condition_combo_box->itemData(ui_.condition_combo_box->currentIndex()).value<Element>();
    skiros_wm::Element sub = ui_.condition_sub_combo_box->itemData(ui_.condition_sub_combo_box->currentIndex()).value<Element>();
    skiros_wm::Element obj = ui_.condition_obj_combo_box->itemData(ui_.condition_obj_combo_box->currentIndex()).value<Element>();
    if(!wm_condition.hasProperty("hasDesiredState"))
        return;
    wm_condition.properties("hasDesiredState").setValue(true);
    if(sub.id()>=0) wm_condition.properties("hasSubject").setValue(sub.toUrl());
    else wm_condition.properties("hasSubject").setValue(sub.label());
    if(obj.type()!="Undefined")
    {
        if(obj.id()>=0) wm_condition.properties("hasObject").setValue(obj.toUrl());
        else wm_condition.properties("hasObject").setValue(obj.label());
    }
    else wm_condition.properties("hasObject").setValue("");
    goal_model_.insertRow(QConditionHolder(wm_condition), goal_model_.rowCount(QModelIndex()));
    //ui_.goal_table_view->resizeColumnsToContents();
    ui_.goal_table_view->resizeRowsToContents();
    ui_.goal_table_view->scrollToBottom();
}

void SkirosGui::removeConditionButtonClicked()
{
    if(!goal_model_.rowCount(QModelIndex()))return;
    QItemSelectionModel * sel_m = ui_.goal_table_view->selectionModel();
    if(sel_m->hasSelection()) //check if has selection
    {
        int index = sel_m->currentIndex().row(); // return selected row(s)
        goal_model_.removeRow(index);
    }
    else goal_model_.removeRow(goal_model_.rowCount(QModelIndex())-1);
}

//World model tab

void SkirosGui::saveSceneButtonClicked()
{
    skiros_msgs::WmSceneLoadAndSave msg;
    msg.request.action = msg.request.SAVE;
    msg.request.filename = ui_.sceneFile_lineEdit->text().toStdString();
    if(scene_load_save_.call(msg))
    {
        if(!msg.response.ok)
            FERROR("[saveScene] Failed to save scene");
    }
    else
        FERROR("[saveScene] Failed to call service");
}

void SkirosGui::loadSceneButtonClicked()
{
    skiros_msgs::WmSceneLoadAndSave msg;
    msg.request.action = msg.request.LOAD;
    msg.request.filename = ui_.sceneFile_lineEdit->text().toStdString();
    if(scene_load_save_.call(msg))
    {
        if(!msg.response.ok)
            FERROR("[loadScene] Failed to load scene");
    }
    else
        FERROR("[loadScene] Failed to call service");
}

std::string getFirstFrameId(skiros_wm::WorldModelInterface & wm, skiros_wm::Element e)
{
    skiros_wm::Element parent = e;
    while(!parent.hasProperty(data::FrameId) && parent.id() > 0) parent = wm.getParentElement(parent);
    return parent.properties(data::FrameId).getValue<std::string>();
}

void SkirosGui::addWmElementButtonClicked()
{
    QItemSelectionModel * sel_m = ui_.world_model_tree_view->selectionModel();
    skiros_wm::Element sel_elem;
    if(sel_m->hasSelection()) //check if has selection
        sel_elem = wm_model_ptr_->getElement(sel_m->currentIndex());
    else
        sel_elem = wm_model_ptr_->getRoot();

    skiros_wm::Element to_add;
    CustomDialog d("Create element", widget_);
    d.initSkirosInterfaces(wm_ptr_, interactive_m_server_);
    d.addSkirosElementCreation(to_add);
    d.exec();                    // Execution stops here until user closes dialog
    if(d.wasCancelled())
        return;

    CustomDialog d2("Edit element", widget_);
    d2.initSkirosInterfaces(wm_ptr_, interactive_m_server_);
    d2.addSkirosElementEdit(to_add, getFirstFrameId(*wm_ptr_, sel_elem));
    d2.exec();                    // Execution stops here until user closes dialog
    if(d2.wasCancelled())
        return;
    interactive_m_server_->clear();
    interactive_m_server_->applyChanges();

    if(!wm_ptr_->addElement(to_add, sel_elem.id(), relation::contain))
        ROS_ERROR("[addWmElementButtonClicked] Error while adding element on world model.");
}

void SkirosGui::modifyWmElementButtonClicked()
{
    QItemSelectionModel * sel_m = ui_.world_model_tree_view->selectionModel();
    if(sel_m->hasSelection()) //check if has selection
    {
        skiros_wm::Element sel_elem = wm_model_ptr_->getElement(sel_m->currentIndex());
        if(sel_elem.id()==0)
        {
            printFadingMessage("Can't modify root node.");
            return;
        }
        CustomDialog d("Edit element", widget_);
        d.initSkirosInterfaces(wm_ptr_, interactive_m_server_);
        d.addSkirosElementEdit(sel_elem, getFirstFrameId(*wm_ptr_, wm_ptr_->getParentElement(sel_elem)));
        d.exec();                    // Execution stops here until user closes dialog
        interactive_m_server_->clear();
        interactive_m_server_->applyChanges();

        if(d.wasCancelled())
            return;

        if(!wm_ptr_->updateElement(sel_elem))
            ROS_ERROR("[modifyWmElementButtonClicked] Error while updating element on world model.");
    }
}

void SkirosGui::removeWmElementButtonClicked()
{
    QItemSelectionModel * sel_m = ui_.world_model_tree_view->selectionModel();
    if(sel_m->hasSelection()) //check if has selection
    {
        skiros_wm::Element sel_elem = wm_model_ptr_->getElement(sel_m->currentIndex());
        if(sel_elem.id()==0)
            return;
        if(wm_ptr_->removeElement(sel_elem.id()))
            wm_model_ptr_->removeElement(sel_m->currentIndex());
    }
}

void SkirosGui::updateWmTree(skiros_wm::WorldGraph graph, int root)
{
    std::vector<skiros_wm::ChildsPair> childs = graph.getChildElements(root);
    for(skiros_wm::ChildsPair pair : childs)
    {
        int child_id = wm_model_ptr_->addElement(pair.second, root, pair.first);
        updateWmTree(graph, child_id);
    }
}

void SkirosGui::onWmTreeSelectionChanged(const QModelIndex & current, const QModelIndex & previous)
{
    skiros_wm::Element sel_elem = wm_model_ptr_->getElement(current);
    //Clear
    while(ui_.tableWidget_wm_object->rowCount()>0)
        ui_.tableWidget_wm_object->removeRow(0);
    //ui_.tableWidget_wm_object->setHorizontalHeaderItem(0, new QTableWidgetItem(sel_elem.toUrl().c_str()));
    int i=0;
    for(auto p : sel_elem.properties())
    {
        ui_.tableWidget_wm_object->insertRow(i);
        ui_.tableWidget_wm_object->setItem(i++, 0, new QTableWidgetItem(p.second.printState().c_str()));
    }
    ui_.tableWidget_wm_object->resizeRowsToContents();
    while(ui_.tableWidget_wm_relations->rowCount()>0)
        ui_.tableWidget_wm_relations->removeRow(0);
    ui_.tableWidget_wm_relations->setHorizontalHeaderItem(0, new QTableWidgetItem("Relations"));
    auto rels = wm_ptr_->queryRelation(sel_elem.id(), "conditionProperty", -1);
    i=0;
    for(skiros_wm::RelationType rel : rels)
    {
        ui_.tableWidget_wm_relations->insertRow(i);
        ui_.tableWidget_wm_relations->setItem(i++, 0, new QTableWidgetItem((rel.predicate()+" "+wm_ptr_->getElement(rel.object_id()).printState("", false)).c_str()));
    }
    ui_.tableWidget_wm_relations->resizeRowsToContents();
    /*std::stringstream ss(wm_ptr_->getContextRelations(sel_elem.toUrl()));
    for (std::array<char, 50> a; ss.getline(&a[0], 50, '\n'); ) {
        ui_.tableWidget_wm_relations->insertRow(i);
        ui_.tableWidget_wm_relations->setItem(i++, 0, new QTableWidgetItem(&a[0]));
    }*/
}

//Module tab
void SkirosGui::exeModuleButtonClicked()
{
    if (curr_robot_=="" || curr_module_== "")return;
    skiros_skill::SkillManagerInterfacePtr mgr = skill_interface_ptr->getSkillMgrsMap().find(curr_robot_)->second;
    skiros_common::ParamMap params = mgr->getParams(curr_module_);
    getParams(ui_.gridLayout_modules, params);
    module_exe_id_ = mgr->exeModule(curr_module_, params, skill_exe_author, boost::bind(&SkirosGui::moduleDoneCb, this, _1));
    if(module_exe_id_<0)
        FERROR("[SkirosGui::exeModuleButtonClicked] Failed to execute module.");
}

void SkirosGui::stopModuleButtonClicked()
{
    if (curr_robot_=="" || curr_module_== "")return;
    skiros_skill::SkillManagerInterfacePtr mgr = skill_interface_ptr->getSkillMgrsMap().find(curr_robot_)->second;
    if(!mgr->stopModule(module_exe_id_, curr_module_, skill_exe_author))
        FERROR("[SkirosGui::stopModuleButtonClicked] Failed to stop module.");
}

void SkirosGui::moduleDoneCb(const skiros_msgs::ModuleStatus &result)
{
    std::ostringstream output;
    output << "[" << result.status
             << "]: "  << "[" << result.progress_code << "]" <<  result.progress_description;
    ui_.module_result_label->setText(output.str().c_str());
}

void SkirosGui::moduleIndexChanged(int index)
{
    if (curr_robot_=="")return;
    curr_module_= ui_.module_combo_box->itemText(index).toStdString();
    if (curr_module_=="")return;
    if(skill_interface_ptr->find(curr_robot_)==skill_interface_ptr->getSkillMgrsMap().end())
      FERROR("[SkirosGui::moduleIndexChanged] Couldn't find the robot " << curr_robot_ << " in list.");
    skiros_skill::SkillManagerInterfacePtr mgr = skill_interface_ptr->getSkillMgrsMap().find(curr_robot_)->second;
    skiros_common::ParamMap params = mgr->getParams(curr_module_);
    QLayoutItem *child;
    while ((child = ui_.gridLayout_modules->takeAt(0)) != 0)
    {
      child->widget()->hide();
      delete child;
    }
    int i=0;
    for(auto p : params)
    {
        if(advanced_modality_ || p.second.specType()==skiros_common::online || p.second.specType()==skiros_common::optional)
            addParameter(ui_.gridLayout_modules, i++, p.second);
    }
}

//Task tab

void SkirosGui::addSkillButtonClicked()
{
    //Get skill info
    if (curr_robot_=="" || curr_skill_== "")return;
    skiros_skill::SkillManagerInterfacePtr mgr = skill_interface_ptr->getSkillMgrsMap().find(curr_robot_)->second;
    skiros_common::ParamMap params = mgr->getParams(curr_skill_);
    getParams(ui_.gridLayout_skills, params);
    int index = -1;
    //TODO: make the skill selection properly
    /*QItemSelectionModel * sel_m = ui_.skills_table_view->selectionModel();
    if(sel_m->hasSelection()) //check if has selection
        index = sel_m->currentIndex().row(); // return row index*/
    //Contact the task manager
    //TODO: check consistancy of curr skill list with task manager
    skiros_msgs::TmModifyTask msg;
    msg.request.author = skill_exe_author;
    msg.request.action = msg.request.ADD;
    msg.request.robot = curr_robot_;
    msg.request.skill.type = wm_ptr_->getType(curr_skill_);
    msg.request.skill.name = curr_skill_;
    msg.request.index = index;
    msg.request.skill.parameters_in = skiros_common::utility::serializeParamMap(params);

    if(!task_modify_.call(msg))
    {
        FERROR("[SkirosGui::addSkillButtonClicked] Fail to contact service " << task_modify_.getService());
        return;
    }
    if(msg.response.return_code<=0)
    {
        FERROR("[SkirosGui::addSkillButtonClicked] Got error while adding skill: " << msg.response.return_code);
        return;
    }

    if(index==-1)index = skill_model_.rowCount(QModelIndex());
    //Add row
    skill_model_.insertRow(QSkillHolder(curr_robot_, msg.request.skill.type, curr_skill_, params), index);
    //ui_.skills_table_view->showRow(skill_model_.rowCount(sel_m->currentIndex()));
    ui_.skills_table_view->resizeColumnsToContents();
    ui_.skills_table_view->resizeRowsToContents();
    ui_.skills_table_view->horizontalHeader()->setStretchLastSection(true);
    ui_.skills_table_view->scrollToBottom();
}

void SkirosGui::removeSkillButtonClicked()
{
    int index = -1;
    if(!skill_model_.rowCount(QModelIndex()))return;
    QItemSelectionModel * sel_m = ui_.skills_table_view->selectionModel();
    if(sel_m->hasSelection()) //check if has selection
        index = sel_m->currentIndex().row(); // return selected row(s)

    //Contact the task manager
    //TODO: check consistancy of curr skill list with task manager
    skiros_msgs::TmModifyTask msg;
    msg.request.author = skill_exe_author;
    msg.request.action = msg.request.REMOVE;
    msg.request.index = index;

    if(!task_modify_.call(msg))
    {
        FERROR("[SkirosGui::removeSkillButtonClicked] Fail to contact service " << task_modify_.getService());
        return;
    }
    if(msg.response.return_code<=0)
    {
        FERROR("[SkirosGui::removeSkillButtonClicked] Got error while removing skill: " << msg.response.return_code);
        return;
    }
    //Remove row
    skill_model_.removeRow(index);
}

void SkirosGui::skillIndexChanged(int index)
{
    if (curr_robot_=="")return;
        curr_skill_= ui_.skill_combo_box->itemText(index).toStdString();
    QLayoutItem *child;
    while ((child = ui_.gridLayout_skills->takeAt(0)) != 0)
    {
      child->widget()->hide();
      delete child;
    }
    if (curr_skill_=="")return;
    if(skill_interface_ptr->find(curr_robot_)==skill_interface_ptr->getSkillMgrsMap().end())
      FERROR("[SkirosGui::moduleIndexChanged] Couldn't find the robot " << curr_robot_ << " in list.");
    skiros_skill::SkillManagerInterfacePtr mgr = skill_interface_ptr->getSkillMgrsMap().find(curr_robot_)->second;
    skiros_common::ParamMap params = mgr->getParams(curr_skill_);
    int i=0;
    for(auto p : params)
    {
      if(p.second.specType()!=skiros_common::planning && (advanced_modality_ || p.second.specType()==skiros_common::online  || p.second.specType()==skiros_common::optional))
          addParameter(ui_.gridLayout_skills, i++, p.second);
    }
}

void SkirosGui::updateTask()
{
    skiros_msgs::TmQueryTask msg;
    if(task_query_.call(msg))
    {
        skill_model_.clear();
        for(int i=0;i<msg.response.task_robots.size();i++)
        {
            skill_model_.insertRow(QSkillHolder(msg.response.task_robots[i].c_str(),
                                                msg.response.task_skills[i].type.c_str(),
                                                msg.response.task_skills[i].name.c_str(),
                                                skiros_common::utility::deserializeParamMap(msg.response.task_skills[i].parameters_in)),
                                                skill_model_.rowCount(QModelIndex()));
        }
        ui_.skills_table_view->resizeRowsToContents();
        ui_.skills_table_view->resizeColumnsToContents();
        ui_.skills_table_view->scrollToBottom();
    }
}


//Logging tab

void SkirosGui::startLogButtonClicked()
{
    logging_ = !logging_;
    if(logging_)
    {
        ui_.startstoplog_button->setText("Stop recording");
        ui_.logFile_lineEdit->setDisabled(1);
    }
    else
    {
        saveLog();
        ui_.startstoplog_button->setText("Start recording");
        ui_.logFile_lineEdit->setDisabled(0);
    }
}

void SkirosGui::logFileChanged()
{
    std::string file = ui_.logFile_lineEdit->text().toStdString();
    if (file.find(".txt")==std::string::npos || file.find("/")==std::string::npos)
    {
        FERROR("[SkirosGui::logFileChanged] File name not valid (missing any '/' and '.txt' at the end): " << file);
        return;
    }
    std::string path = file.substr(0, file.find_last_of('/'));
    QDir dir(path.c_str());
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    std::ifstream t(file.c_str());
    if(t.is_open())
    {
        std::string str((std::istreambuf_iterator<char>(t)),
                         std::istreambuf_iterator<char>());
        ui_.log_textEdit->setText(str.c_str());
        t.close();
    }
    else FERROR("[SkirosGui::logFileChanged] Unable to open file " << file);
}

void SkirosGui::saveLog()
{
    auto path = ui_.logFile_lineEdit->text().toStdString();
    ofstream myfile(path);
    if (myfile.is_open())
    {
      myfile << ui_.log_textEdit->toPlainText().toStdString();
      myfile.close();
    }
    else FERROR("[SkirosGui::saveLog] Unable to open file " << path);
}

// General functions

void SkirosGui::refreshTimerCb()
{
    ///Refresh robot combo box
    if(skill_interface_ptr->hasChanged())
    {
        ui_.robot_combo_box->clear();
        robot_monitor_sub_.clear();
        for(auto pair1 : skill_interface_ptr->getSkillMgrsMap())
        {
            auto robot = pair1.first;
            ui_.robot_combo_box->addItem(robot.c_str());
            robot_monitor_sub_.push_back(nh_ptr_->subscribe(robot+skiros_config::skill_monitor_tpc_name,10, &SkirosGui::robotFeedbackReceived, this));
        }
    }
    ///Refresh skill\modules combo boxes
    if(curr_robot_!="")
    {
        skiros_skill::SkillManagerInterfacePtr temp = skill_interface_ptr->find(curr_robot_)->second;
        if(temp->hasChanged())
        {
            ui_.module_combo_box->clear();
            for(auto pair2  : temp->getModuleList())
            {
                ui_.module_combo_box->addItem(pair2.first.c_str());
            }
            ui_.skill_combo_box->clear();
            for(auto pair2  : temp->getSkillList())
            {
                ui_.skill_combo_box->addItem(pair2.first.c_str());
            }
        }
    }
    ///Refresh world model tree
    if(wm_ptr_->hasChanged())
    {
        boost::mutex::scoped_lock lock(wm_tree_mux_);
        skiros_wm::WorldGraph graph;
        wm_ptr_->getBranch(graph, 0, relation::Str[relation::spatiallyRelated]);
        wm_model_ptr_->clear();
        wm_model_ptr_->setRoot(graph.getElement(0));
        auto id = wm_model_ptr_->addElement(graph.getElement(0), -1, "");
        updateWmTree(graph, id);
        ui_.world_model_tree_view->expandAll();
    }
}

void SkirosGui::getParams(QGridLayout* layout, skiros_common::ParamMap & param_map)
{
    int i=0;
    for(auto & pair : param_map)
    {
        if(pair.second.isType(skiros_common::planning) || (!advanced_modality_ && !pair.second.isType(skiros_common::online) && !pair.second.isType(skiros_common::optional)))
        {
            auto & p = pair.second;
            if(p.hasValueType(skiros_wm::Element()))
            {
                std::vector<skiros_wm::Element> v;
                if(p.state() == skiros_common::specified)
                {
                    skiros_wm::Element temp = p.getValue<skiros_wm::Element>();
                    v = wm_ptr_->resolveElement(temp);
                }
                else
                {
                    v = wm_ptr_->resolveElement(skiros_wm::Element(concept::Str[concept::Unknown]));
                }
                if(v.size()>0)
                    p.setValue(v[0]);
            }
            continue;
        }
        QLabel * label = qobject_cast<QLabel*>(layout->itemAtPosition(i, 0)->widget());
        std::string key = label->text().toStdString();
        skiros_common::ParamMap::iterator it = param_map.find(key.substr(0, key.find_first_of(' ')));
        if(it==param_map.end())
            continue;
        if(it->second.type()==typeid(bool))
        {
            QCheckBox * widget = qobject_cast<QCheckBox*>(layout->itemAtPosition(i, 1)->widget());
            it->second.setValue(widget->isChecked());
        }
        else if (it->second.type()==typeid(std::string) || it->second.type()==typeid(int) || it->second.type()==typeid(double) || it->second.type()==typeid(float))
        {
            QLineEdit * widget = qobject_cast<QLineEdit*>(layout->itemAtPosition(i, 1)->widget());
            it->second.setValue(widget->text().toStdString());
        }
        else if(it->second.type()==typeid(skiros_wm::Element))
        {
            QComboBox * widget = qobject_cast<QComboBox*>(layout->itemAtPosition(i, 1)->widget());
            int element_id = widget->itemData(widget->currentIndex()).toInt();
            if(element_id>0)
            {
                skiros_wm::Element e = wm_ptr_->getElement(element_id);
                it->second.setValue(e);
            }
        }
        i++;
    }
}

void SkirosGui::addParameter(QGridLayout* layout, int row, skiros_common::Param p)
{
    //FDEBUG("Add " << p.printState());
    layout->setColumnStretch(2, 10);
    layout->setRowMinimumHeight(row, 2);
    QString description(p.name().c_str());
    auto temp = p.key();
    if(p.specType()==skiros_common::optional)
        temp += " (OPTIONAL)";
    QLabel* key = new QLabel(temp.c_str());
    key->setToolTip(description);
    if(p.type()==typeid(bool))
    {
        QCheckBox* check_box = new QCheckBox;//Elements
        check_box->setToolTip(description);
        if(p.isSpecified())check_box->setChecked(p.getValue<bool>());
        layout->addWidget(key, row, 0);
        layout->addWidget(check_box, row, 1);
    }
    else if (p.type()==typeid(std::string) || p.type()==typeid(int) || p.type()==typeid(double) || p.type()==typeid(float))
    {
        QLineEdit* line_edit = new QLineEdit;
        line_edit->setToolTip(description);
        if(p.isSpecified())line_edit->setText(p.getValueStr().c_str());
        layout->addWidget(key, row, 0);
        layout->addWidget(line_edit, row, 1);
    }
    else if(p.type()==typeid(skiros_wm::Element))
    {
        QComboBox* combo_box = new QComboBox;//Elements
        combo_box->setToolTip(description);
        layout->addWidget(key, row, 0);
        layout->addWidget(combo_box, row, 1);

        //Create a function for this
        std::vector<skiros_wm::Element> v;
        if(p.state() == skiros_common::specified)
        {
            skiros_wm::Element temp = p.getValue<skiros_wm::Element>();
            v = wm_ptr_->resolveElement(temp);
        }
        else
        {
              v = wm_ptr_->resolveElement(skiros_wm::Element(concept::Str[concept::Unknown]));
        }
        if(p.specType()==skiros_common::optional)combo_box->addItem("", QVariant(-1));
        for(skiros_wm::Element e : v)
        {
            combo_box->addItem(e.printState("", false).c_str(), QVariant(e.id()));
        }
    }
}

void SkirosGui::modalityButtonClicked()
{
    advanced_modality_ = ui_.modality_checkBox->isChecked();
    skillIndexChanged(ui_.skill_combo_box->currentIndex());
    moduleIndexChanged(ui_.module_combo_box->currentIndex());
}

void SkirosGui::exeTaskButtonClicked()
{
    skiros_msgs::TmTaskExe msg;
    msg.exe = true;
    msg.iterate = ui_.checkBox_iterate->isChecked();
    task_exe_pub_.publish(msg);
}

void SkirosGui::stopTaskButtonClicked()
{
    skiros_msgs::TmTaskExe msg;
    msg.exe = false;
    task_exe_pub_.publish(msg);
}

void SkirosGui::setCurrentRobot(std::string name)
{
    curr_robot_ = name;
    ///Refresh skill\modules combo boxes
    if(curr_robot_!="")
    {
        skiros_skill::SkillManagerInterfacePtr temp = skill_interface_ptr->find(curr_robot_)->second;
        ui_.module_combo_box->clear();
        for(auto pair2  : temp->getModuleList())
        {
            ui_.module_combo_box->addItem(pair2.first.c_str());
        }
        ui_.skill_combo_box->clear();
        for(auto pair2  : temp->getSkillList())
        {
            ui_.skill_combo_box->addItem(pair2.first.c_str());
        }
    }
}

void SkirosGui::robotIndexChanged(int index)
{
    setCurrentRobot(ui_.robot_combo_box->itemText(index).toStdString());
}

void SkirosGui::robotMonitorCb(const skiros_msgs::ModuleStatus & msg)
{
    int current_row = ui_.tableWidget_output->rowCount();
    ui_.tableWidget_output->insertRow(current_row);
    ui_.tableWidget_output->setItem(current_row, 0, new QTableWidgetItem(msg.module.name.c_str()));
    ui_.tableWidget_output->setItem(current_row, 1, new QTableWidgetItem(msg.status.c_str()));
    ui_.tableWidget_output->setItem(current_row, 2, new QTableWidgetItem(std::to_string(msg.progress_code).c_str()));
    ui_.tableWidget_output->setItem(current_row, 3, new QTableWidgetItem(msg.progress_description.c_str()));
    ui_.tableWidget_output->resizeRowsToContents();
    ui_.tableWidget_output->scrollToBottom();
    ui_.tableWidget_output->showRow(current_row);
    if(logging_ && (msg.status=="preempted" || msg.status=="error" ||msg.status=="terminated"))
    {
        std::stringstream ss;
        time_t current_time = time(NULL);
        struct tm * now = localtime( & current_time);
        std::stringstream time_date_stamp;
        time_date_stamp << (now->tm_year+1900) << '-'
                << (now->tm_mon +1) << '-'
                << (now->tm_mday) << '_'
                << now->tm_hour << ':'
                << now->tm_min << ':'
                << now->tm_sec;

        ss << time_date_stamp.str() << " " << msg.controller << " "
           << msg.module.name  << " "
           << msg.progress_code << " "
           << "'" << msg.progress_description << "'"
           << " " << msg.progress_seconds << " ";
        /*
        for(auto pair : onlineParams)
        {
            skiros_common::Param p = pair.second;
            if(p.type()==typeid(skiros_wm::Element))
            {
                ss << "'" << p.key() << ":" << p.getValue<skiros_wm::Element>().type() << "' ";
            }
        }*/
        ui_.log_textEdit->append(ss.str().c_str());
    }
}

void SkirosGui::taskMonitorCb(const skiros_msgs::TmMonitor & msg)
{
    if(msg.action=="started" || msg.action=="terminated" || msg.action=="error" )
    {
        //Ignore (redundant)
    }
    else if(msg.action=="modified" && msg.author!=skill_exe_author)
    {
        updateTask();
    }
    else if(logging_)
    {
        std::stringstream ss;
        time_t current_time = time(NULL);
        struct tm * now = localtime( & current_time);
        std::stringstream time_date_stamp;
        time_date_stamp << (now->tm_year+1900) << '-'
                << (now->tm_mon +1) << '-'
                << (now->tm_mday) << '_'
                << now->tm_hour << ':'
                << now->tm_min << ':'
                << now->tm_sec;

        ss << time_date_stamp.str() << " " << msg.author << " "
           << msg.action << " "
           << msg.progress_code << " "
           << "'" << msg.progress_description << "'";
        if(msg.progress_seconds>0)
            ss << " " << msg.progress_seconds << " ";
        else
            ss << " - ";
        ui_.log_textEdit->append(ss.str().c_str());

    }
}



}

PLUGINLIB_EXPORT_CLASS(skiros_gui::SkirosGui, rqt_gui_cpp::Plugin)
