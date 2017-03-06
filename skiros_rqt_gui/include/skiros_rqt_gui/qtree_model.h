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
#ifndef q_tree_model_h
#define q_tree_model_h

#include <QVariant>
#include <QAbstractItemModel>
#include <QtGlobal>
#include <skiros_world_model/world_element.h>

namespace skiros_gui {

class TreeItem
{
public:
    explicit TreeItem(const skiros_wm::Element &data, TreeItem *parentItem, std::string relation="");
    ~TreeItem();

    void appendChild(TreeItem *child);

    void removeChild(TreeItem* child)
    {
        QList<TreeItem*>::iterator it;
        for(it = m_childItems.begin();it!=m_childItems.end();it++)
        {
            if(*it==child)
                break;
        }
        if(it!=m_childItems.end())
        {
            m_childItems.erase(it);
            delete child;
        }
    }

    void removeChild(int row)
    {
        QList<TreeItem*>::iterator it = m_childItems.begin();
        for(int i=0;i<row;i++)
            it++;
        delete this->child(row);
        m_childItems.erase(it);
    }

    int getId() const {return m_itemData.id();}
    std::string parentRelation() const{return parent_relation_;}
    TreeItem *child(int row);
    int childCount() const;
    skiros_wm::Element data() const;
    int row() const;
    TreeItem *parentItem();

private:
    //Tree connections
    QList<TreeItem*> m_childItems;
    TreeItem *m_parentItem;
    //Data
    std::string parent_relation_;
    skiros_wm::Element m_itemData;
};

class TreeModel : public QAbstractItemModel
{
    Q_OBJECT

public:
    explicit TreeModel(const skiros_wm::Element &data, QObject *parent=NULL);
    ~TreeModel();

    //Overloaded functions
    QVariant data(const QModelIndex &index, int role) const;
    Qt::ItemFlags flags(const QModelIndex &index) const;
    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const;
    QModelIndex index(int row, int column,
                      const QModelIndex &parent = QModelIndex()) const;
    QModelIndex parent(const QModelIndex &index) const;
    int rowCount(const QModelIndex &parent = QModelIndex()) const;
    int columnCount(const QModelIndex &parent = QModelIndex()) const;

    void clear()
    {
        beginResetModel();
        id_map_.clear();
        endResetModel();
    }

    void setRoot(const skiros_wm::Element &data)
    {
        rootItem = new TreeItem(data, NULL);
        id_map_.insert(IdPair(-1, createIndex(0, 0, rootItem)));
    }


    skiros_wm::Element getRoot()
    {
        return rootItem->data();
    }

    skiros_wm::Element getElement(const QModelIndex &index)
    {
        return static_cast<TreeItem*>(index.internalPointer())->data();
    }

    //My functions
    void removeElement(const QModelIndex &index)
    {
        if (!index.isValid())return;

        TreeItem *childItem = static_cast<TreeItem*>(index.internalPointer());
        TreeItem *parentItem = childItem->parentItem();

        if (parentItem == rootItem)return;
        id_map_.erase(childItem->getId());
        removeRow(index.row(), parent(index));
        parentItem->removeChild(childItem);
    }

    int addElement(const skiros_wm::Element &data, int parent_id, std::string relation)
    {
        return addElement(data, getElementIndex(parent_id), relation);
    }


private:

    QModelIndex getElementIndex(int id)
    {
        IdMap::iterator it = id_map_.find(id);
        if(it!=id_map_.end())
            return it->second;
        else
            return QModelIndex();
    }

    int addElement(const skiros_wm::Element &data, QModelIndex parent_index, std::string relation)
    {
        if (!parent_index.isValid())
            return -1;

        TreeItem* parent = static_cast<TreeItem*>(parent_index.internalPointer());
        insertRow(parent->childCount(), parent_index);
        TreeItem *ptr = new TreeItem(data, parent, relation);
        parent->appendChild(ptr);
        QModelIndex index = createIndex(ptr->row(), 0, ptr);
        id_map_.insert(IdPair(ptr->getId(), index));
        return ptr->getId();
    }

    typedef std::map<int, QModelIndex> IdMap;
    typedef std::pair<int, QModelIndex> IdPair;
    //Support fast item research via IDs
    IdMap id_map_;
    //The root node
    TreeItem *rootItem;
};


}

#endif // q_tree_model_h
