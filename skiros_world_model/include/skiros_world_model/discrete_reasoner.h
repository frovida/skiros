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

#ifndef DISCRETE_REASONER_HPP
#define DISCRETE_REASONER_HPP

#include "skiros_world_model/world_element.h"
#include "skiros_world_model/owl_world_model.h"

namespace skiros_wm
{

typedef std::multimap<std::string, std::pair<std::string, std::string> > ReasonerDataMap;
typedef std::pair<std::string, std::pair<std::string, std::string> > ReasonerDataPair;
typedef std::map<std::string, std::pair<std::string, std::string> > RelationsMap;
typedef std::pair<std::string, std::pair<std::string, std::string> > RelationsPair;

/*!
      \brief This virtual class is the template for all discrete reasoners.
        A discrete reasoner create a link between sensor data and semantic data.
        It differs from semantic reasoners because it works at a lower level.
        It relates clusters of data to elements and allow to reason on them to classify and identify objects.
        Ex. A spatial reasoner relate data "Size" to the elements. When queried for
        compute similarities it make use of "Size" data to compute a correlation coefficient.
        Reasoners shall be imported as plugins using pluginlib.
    */
class DiscreteReasoner
{
public:
    virtual ~DiscreteReasoner(){}
    /*!
     * \brief This routine is called from the World Model Node at boot and should be used to keep the wm updated. Doesn't have to return
     * \param wm a pointer to the world model
     */
    virtual void execute(owl::WorldModel * wm) {}
    /*!
    * \brief Return semantic relations between two objects, that can be inferred with the reasoner
    * \param subject
    * \param object
    * \return a map with semantic relations as keys, and literals as values
    */
    virtual RelationsMap computeRelations(skiros_wm::Element &subject, skiros_wm::Element &object) = 0;
    /*!
    * \brief Compare two elements to determine the coefficient of istance similarity (identification)
    * \param lhs first element
    * \param rhs second element
    * \return A float between 0 and 1. 0 means NO similarity, 1 means full similarity
    */
    virtual float computeIstanceSimilarity(skiros_wm::Element &lhs, skiros_wm::Element &rhs) = 0;
    /*!
    * \brief Compare two elements to determine the coefficient of class similarity (classification)
    * \param lhs first element
    * \param rhs second element
    * \return A float between 0 and 1. 0 means NO similarity, 1 means full similarity
    */
    virtual float computeClassSimilarity(skiros_wm::Element &lhs, skiros_wm::Element &rhs) = 0;
    /*!
    * \brief Convert user data to reasoner data and store it into the element
    * \param e the element where the data will be stored
    * \param any the data to be stored
    * \param set_code a code defining which data want to store. For the available codes refer to the documentation of the specific reasoner
    * \return
    */
    virtual bool storeData(skiros_wm::Element & e, boost::any any, std::string  set_code="") = 0;
    template<class T>
    //! \brief Templated version of storeData
    bool storeData(skiros_wm::Element & e, T any, std::string set_code="")
    {
        return storeData(e, boost::any(any), set_code);
    }
    template<class T>
    //! \brief Templated version of storeData
    bool storeData(skiros_wm::Element & e, T any, skiros_config::owl::data::DataType set_code)
    {
        return storeData(e, boost::any(any), skiros_config::owl::data::Str[set_code]);
    }
    /*!
    * \brief Convert user data to reasoner data and store it into given element
    * \param e the element from which the data must be extracted
    * \param get_code a code defining which data want to get. For the available codes refer to the documentation of the specific reasoner
    * \return
    */
    virtual boost::any getData(skiros_wm::Element & e, std::string  get_code="") = 0;
    template<class T>
    //! \brief Templated version of getData
    T getData(skiros_wm::Element & e, std::string get_code="")
    {
        return boost::any_cast<T>(getData(e, get_code));
    }
    template<class T>
    //! \brief Templated version of getData
    T getData(skiros_wm::Element & e, skiros_config::owl::data::DataType get_code)
    {
        return getData<T>(e, skiros_config::owl::data::Str[get_code]);
    }
    /*!
    * \brief extractOwlData, remove from the element all params that are not convertible to OWL
    * \param e the element to act on. All properties returned in OWL format are removed from the object
    * \return the extracted parameters in a OWL compatible multimap
    */
    virtual ReasonerDataMap extractOwlData(skiros_wm::Element & e) = 0;
    //! \brief Add default reasoner properties to the element
    void addProperties(skiros_wm::Element& e);
    //! \brief Remove default reasoner properties from the element
    void removeProperties(skiros_wm::Element& e);

    std::string getName() { return name_; }
    std::string getType() { return type_; }
protected:
    //! \brief Add default reasoner properties to the element
    virtual void onAddProperties(skiros_wm::Element& e) = 0;
    //! \brief Remove default reasoner properties from the element
    virtual void onRemoveProperties(skiros_wm::Element& e) = 0;

    DiscreteReasoner(){}
    void setName(std::string name) { name_ = name; }
    void setType(std::string type) { type_ = type; }
    std::string name_;
    std::string type_;
};

}
#endif //DISCRETE_REASONER_HPP
