#ifndef WM_UTILITY_H
#define WM_UTILITY_H

#include <vector>

namespace skiros_msgs
{
ROS_DECLARE_MESSAGE(WmElement);
ROS_DECLARE_MESSAGE(WmRelation);
}

namespace skiros_wm
{
class Element;
class RelationType;

//! Utilities to convert element and relations to its corrispective ROS msg and vice-versa
skiros_msgs::WmElement element2msg(skiros_wm::Element e);
skiros_wm::Element msg2element(skiros_msgs::WmElement e);

skiros_msgs::WmRelation relation2msg(RelationType relation);
RelationType msg2relation(skiros_msgs::WmRelation msg);

std::vector<Element> msgs2elements(std::vector<skiros_msgs::WmElement> list);
std::vector<skiros_msgs::WmElement> elements2msgs(std::vector<Element> list);

std::vector<RelationType> msgs2relations(std::vector<skiros_msgs::WmRelation> list);
std::vector<skiros_msgs::WmRelation> relations2msgs(std::vector<RelationType> list);

}

#endif //WM_UTILITY_H
