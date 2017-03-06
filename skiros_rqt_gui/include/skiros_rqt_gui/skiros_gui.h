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

#ifndef skiros_rqt_gui_h
#define skiros_rqt_gui_h

#include <rqt_gui_cpp/plugin.h>
#include <ui_skiros_gui.h>
#include <ros/macros.h>
#include <QList>
#include <QString>
#include <QSize>
#include <QWidget>
#include <QAbstractTableModel>
#include <skiros_rqt_gui/table_models.h>
#include <skiros_rqt_gui/qtree_model.h>
#include <skiros_msgs/WmMonitor.h>
#include <skiros_msgs/TmMonitor.h>
#include <skiros_msgs/ModuleStatus.h>
#include <skiros_skill/skill_layer_interface.h>
#include <boost/thread.hpp>
#include <interactive_markers/interactive_marker_server.h>


Q_DECLARE_METATYPE(skiros_msgs::ModuleStatus);
Q_DECLARE_METATYPE(skiros_msgs::TmMonitor);
Q_DECLARE_METATYPE(skiros_wm::Element);

namespace skiros_gui {

/*!
 * \brief SkiROS Graphical User Interface. Plugin for rqt
 */
class SkirosGui
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  SkirosGui();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  /*virtual void notify()
  {

  }*/
signals:
  //NOTE: ALL ROS CALLBACKs THAT MODIFY THE GUI HAVE TO BE QT SIGNALS AND THEN CONNECTED TO A CALLBACK FUNCTION
  void robotFeedbackReceived(const skiros_msgs::ModuleStatus & msg);
  void tmFeedbackReceived(const skiros_msgs::TmMonitor & msg);

protected slots:
  //General
  void refreshTimerCb();
  void robotMonitorCb(const skiros_msgs::ModuleStatus & msg);
  void robotIndexChanged(int index);
  void getParams(QGridLayout* layout, skiros_common::ParamMap & param_map);
  void addParameter(QGridLayout *layout, int row, skiros_common::Param p);
  void modalityButtonClicked();
  void exeTaskButtonClicked();
  void stopTaskButtonClicked();

  //Goal tab
  void conditionIndexChanged(int index);
  void refreshButtonClicked();
  void taskPlanButtonClicked();
  void addConditionButtonClicked();
  void removeConditionButtonClicked();

  //World model tab
  void onWmTreeSelectionChanged(const QModelIndex & current, const QModelIndex & previous);
  void removeWmElementButtonClicked();
  void modifyWmElementButtonClicked();
  void addWmElementButtonClicked();
  void saveSceneButtonClicked();
  void loadSceneButtonClicked();

  //Task tab
  void addSkillButtonClicked();
  void removeSkillButtonClicked();
  void skillIndexChanged(int index);
  void taskMonitorCb(const skiros_msgs::TmMonitor & msg);

  //Modules tab
  void exeModuleButtonClicked();
  void stopModuleButtonClicked();
  void moduleIndexChanged(int index);

  //Logging tab
  void startLogButtonClicked();
  void logFileChanged();

protected:
  void saveLog();
  //Skill layer interface
  void printFadingMessage(std::string msg);
  void moduleDoneCb(const skiros_msgs::ModuleStatus & result);
  int module_exe_id_;
  //World model interface
  void updateWmTree(skiros_wm::WorldGraph graph, int root);
  boost::mutex wm_tree_mux_;
  TreeModel * wm_model_ptr_;
  boost::shared_ptr<skiros_wm::WorldModelInterface> wm_ptr_;
  ros::ServiceClient scene_load_save_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_m_server_;

  //Task manager interface
  void updateTask();
  ros::Publisher task_exe_pub_;
  ros::Subscriber task_monitor_sub_;
  ros::ServiceClient task_modify_;
  ros::ServiceClient task_query_;

  //Goal tab
  QGoalModel goal_model_;
  ros::ServiceClient task_plan_;

  //Refresh timer
  QTimer * timer;
  //Current selections
  std::string curr_skill_;
  std::string curr_module_;
  std::string curr_robot_;
  std::vector<ros::Subscriber> robot_monitor_sub_;
  QSkillModel skill_model_;
  void setCurrentRobot(std::string name);
  boost::shared_ptr<skiros_skill::SkillLayerInterface> skill_interface_ptr;

  //Basic variables
  bool advanced_modality_ = false;
  bool logging_ = false;
  Ui::SkirosGuiWidget ui_;
  QWidget* widget_;
  boost::shared_ptr<ros::NodeHandle> nh_ptr_;
};


}


#endif // skiros_rqt_gui_h
