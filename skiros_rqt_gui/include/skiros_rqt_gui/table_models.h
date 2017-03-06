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

#ifndef TABLE_MODELS_H
#define TABLE_MODELS_H

#include <skiros_world_model/world_element.h>
#include <QAbstractTableModel>

namespace skiros_gui
{
/*!
 * \brief Skill wrapper for the QSkillModel
 */
class QSkillHolder {
    QString type_, name_, robot_;
    skiros_common::ParamMap params_;
public:
    QSkillHolder(const std::string & robot, const std::string & type, const std::string & name, skiros_common::ParamMap params) :
        type_(type.c_str()), name_(name.c_str()), robot_(robot.c_str()), params_(params) {}
    QString type() const { return type_; }
    QString name() const { return name_; }
    QString robot() const { return robot_; }
    skiros_common::ParamMap params() const { return params_; }
};

/*!
 * \brief Model for the skill list view
 */
class QSkillModel : public QAbstractTableModel {
    QList<QSkillHolder> data_;
public:
    QSkillModel(QObject * parent = 0) : QAbstractTableModel(parent) {}
    int rowCount(const QModelIndex &) const { return data_.count(); }
    int columnCount(const QModelIndex &) const { return 2; }
    QVariant data(const QModelIndex &index, int role) const
    {
        const QSkillHolder & skill = data_[index.row()];
        if (role == Qt::ToolTipRole)
        {
            std::stringstream ss;
            ss << skill.name().toStdString() <<":" << std::endl;
            for(auto pair : skill.params())
            {
                ss << pair.second.printState() << std::endl;
            }
            return ss.str().c_str();
        }
        if (role != Qt::DisplayRole && role != Qt::EditRole) return QVariant();
        switch (index.column()) {
        case 0: return skill.name();
        case 1: return skill.robot();
        default:
            return QVariant();
        }
    }
    QVariant headerData(int section, Qt::Orientation orientation, int role) const {
        if (orientation != Qt::Horizontal) return QVariant();
        if (role != Qt::DisplayRole) return QVariant();
        switch (section) {
        case 0: return "Skill name";
        case 1: return "Robot";
        default: return QVariant();
        }
    }

    void clear()
    {
        beginRemoveRows(QModelIndex(), 0, rowCount(QModelIndex())-1);
        data_.clear();
        endRemoveRows();
    }

    void removeRow(int index)
    {
        if(index<0)index = rowCount(QModelIndex())-1;
        beginRemoveRows(QModelIndex(), index, index);
        QList<QSkillHolder>::iterator it = data_.begin();
        for(int i=0;i<index;i++)
            it++;
        data_.erase(it);
        endRemoveRows();
    }

    void insertRow(const QSkillHolder & skill, int index=-1) {
        if(index<0)
        {
            beginInsertRows(QModelIndex(), index, index);
            data_.append(skill);
        }
        else
        {
            beginInsertRows(QModelIndex(), index, index);
            data_.insert(index, skill);
        }
        endInsertRows();
    }
};

/*!
 * \brief Condition wrapper for the QGoalModel
 */
class QConditionHolder {
    skiros_wm::Element data_;
public:
    QConditionHolder(skiros_wm::Element e) :
        data_(e){}
    QString type() const { return data_.type().c_str(); }
    skiros_common::ParamMap params() const { return data_.properties(); }
    skiros_wm::Element & data() {return data_;}
    const skiros_wm::Element data() const {return data_;}
};

/*!
 * \brief Model for the goal list view
 */
class QGoalModel : public QAbstractTableModel {
    QList<QConditionHolder> data_;
public:
    QGoalModel(QObject * parent = 0) : QAbstractTableModel(parent) {}
    int rowCount(const QModelIndex &) const { return data_.count(); }
    int columnCount(const QModelIndex &) const { return 1; }
    QVariant data(const QModelIndex &index, int role) const
    {
        const QConditionHolder & cond = data_[index.row()];
        if (role == Qt::ToolTipRole)
        {
            std::stringstream ss;
            ss << cond.type().toStdString() <<":" << std::endl;
            for(auto pair : cond.params())
            {
                ss << pair.second.printState() << std::endl;
            }
            return ss.str().c_str();
        }
        if (role != Qt::DisplayRole && role != Qt::EditRole) return QVariant();
        switch (index.column()) {
        case 0:
        {
            std::stringstream ss;
            if(!cond.params().find("hasDesiredState")->second.getValue<bool>())
                ss << "not";
            ss << cond.type().toStdString() <<" ";
            ss << cond.params().find("hasSubject")->second.printValue() << " ";
            if(cond.params().find("hasObject")->second.printValue()!=" [ ]")
                ss << cond.params().find("hasObject")->second.printValue();
            return ss.str().c_str();
        }
        default:
            return QVariant();
        }
    }
    QVariant headerData(int section, Qt::Orientation orientation, int role) const {
        if (orientation != Qt::Horizontal) return QVariant();
        if (role != Qt::DisplayRole) return QVariant();
        switch (section) {
        case 0: return "Condition";
        default: return QVariant();
        }
    }

    const QConditionHolder & getRow(int index)
    {
        return data_[index];
    }

    void removeRow(int index=-1)
    {
        if(index<0)index = rowCount(QModelIndex())-1;
        beginRemoveRows(QModelIndex(), index, index);
        QList<QConditionHolder>::iterator it = data_.begin();
        for(int i=0;i<index;i++)
            it++;
        data_.erase(it);
        endRemoveRows();
    }

    void insertRow(const QConditionHolder & cond, int index=-1) {
        if(index<0)
        {
            beginInsertRows(QModelIndex(), index, index);
            data_.append(cond);
        }
        else
        {
            beginInsertRows(QModelIndex(), index, index);
            data_.insert(index, cond);
        }
        endInsertRows();
    }
};


}

#endif // TABLE_MODELS_H
