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

#ifndef PARAM_TYPES_H
#define PARAM_TYPES_H

#include "skiros_config/param_base.h"
#include "skiros_common/logger_sys.h"
#include "skiros_config/declared_uri.h"
#include <pluginlib/class_loader.h>

namespace skiros_config
{
    const char XSD_DOUBLE[] = "xsd:double";
    const char XSD_INT[] = "xsd:int";
    const char XSD_BOOL[] = "xsd:boolean";
    const char XSD_STRING[] = "xsd:string";

    /*!
     * \brief PARAMETERS DOMAIN MODEL
     *  This singleton class collects all information about standard parameters used in the system
     */
    class ParamTypes
    {
    public:
        //------------------ Public methods ------------------------
        static ParamTypes& getInstance()
        {
          static ParamTypes istance;
          return istance;
        }

        ~ParamTypes();

        //! \brief Returns a list of available parameter names
        const std::vector<std::string> types() const;

        /*!
         * \brief check existence of a parameter
         * \param std_param_type the name of the parameter
         * \return true if the name identifies a standard parameter, false otherwise
         */
        bool isStdParam(std::string std_param_type);
        /*!
         * \brief Get the owl string correspondent to the datatype of the parameter
         * \param p a parameter
         * \return The OWL string for the datatype. If the datatype is not supported, it is considered like a string datatype.
         */
        std::string getDataTypeStr(skiros_common::Param p);

        /*!
         * \brief Get a default parameter
         * \param std_param_type the name of a pre-defined param
         * \return a parameter initialized as default
         */
        skiros_common::Param getDefault(std::string std_param_type);
        skiros_common::Param getDefault(owl::data::DataType std_param_type) { return getDefault(owl::data::Str[std_param_type]); }

        /*!
         * \brief Get a parameter with a datatype specified in a string in OWL format
         * \param key the param ID
         * \param data_type the datatype specified with an abbreviated OWL string. Supported input: xsd:double, xsd:int, xsd:boolean, xsd:string
         * \param name an optional description
         * \param param_type the parameter type in skiros (see ParamSpecType)
         * \param size the vector length
         * \return a parameter initialized as from input
         */
        skiros_common::Param getFromString(std::string key,
                                                       std::string data_type,
                                                       std::string name="",
                                                       skiros_common::ParamSpecType param_type=skiros_common::symbolic,
                                                       int size=1);

    private:
        //------------------ Private methods & variables ------------------------
        ParamTypes();
        pluginlib::ClassLoader<skiros_config::ParamBase> * param_loader_;
        boost::shared_ptr<skiros_config::ParamBase> param_ptr_;
        typedef std::pair<std::string, boost::shared_ptr<skiros_config::ParamBase> > ParamTypesPair;
        typedef std::map<std::string, boost::shared_ptr<skiros_config::ParamBase> > ParamTypesMap;
        ParamTypesMap declared_types_;
    };
}
#endif //PARAM_TYPES_H
