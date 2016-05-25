#include "skiros_common/utility.h"
#include "skiros_world_model/utility.h"
#include "skiros_world_model/world_element.h"
#include <boost/foreach.hpp>

namespace skiros_wm
{
  skiros_msgs::WmElement element2msg(skiros_wm::Element e)
  {
    skiros_msgs::WmElement ret;
    ret.id = e.id();
    ret.type = e.type();
    ret.last_update = e.lastUpdate();
    ret.label = e.label();
    ret.s_properties = skiros_common::utility::serializeParamMap(e.properties());
    return ret;
  }

  skiros_wm::Element msg2element(skiros_msgs::WmElement e)
  {
    skiros_wm::Element ret;
    ret.id() = e.id;
    ret.type() = e.type;
    ret.lastUpdate() = e.last_update;
    ret.label() = e.label;
    ret.properties() = skiros_common::utility::deserializeParamMap(e.s_properties);
    return ret;
  }

  skiros_msgs::WmRelation relation2msg(RelationType relation)
  {
      skiros_msgs::WmRelation msg;
      msg.object_id = relation.object_id();
      msg.predicate = relation.predicate();
      msg.subject_id = relation.subject_id();
      return msg;
  }

  std::vector<Element> msgs2elements(std::vector<skiros_msgs::WmElement> list)
  {
      std::vector<Element> to_ret;
      BOOST_FOREACH(skiros_msgs::WmElement msg, list)
      {
          to_ret.push_back(msg2element(msg));
      }
      return to_ret;
  }

  std::vector<RelationType> msgs2relations(std::vector<skiros_msgs::WmRelation> list)
  {
      std::vector<RelationType> to_ret;
      BOOST_FOREACH(skiros_msgs::WmRelation msg, list)
      {
          to_ret.push_back(msg2relation(msg));
      }
      return to_ret;
  }

  RelationType msg2relation(skiros_msgs::WmRelation msg)
  {
      RelationType relation;
      relation.object_id() = msg.object_id;
      relation.predicate() = msg.predicate;
      relation.subject_id() = msg.subject_id;
      return relation;
  }

  std::vector<skiros_msgs::WmElement> elements2msgs(std::vector<Element> list)
  {
      std::vector<skiros_msgs::WmElement> to_ret;
      BOOST_FOREACH(Element msg, list)
      {
          to_ret.push_back(element2msg(msg));
      }
      return to_ret;
  }

  std::vector<skiros_msgs::WmRelation> relations2msgs(std::vector<RelationType> list)
  {
      std::vector<skiros_msgs::WmRelation> to_ret;
      BOOST_FOREACH(RelationType msg, list)
      {
          to_ret.push_back(relation2msg(msg));
      }
      return to_ret;
  }
}
