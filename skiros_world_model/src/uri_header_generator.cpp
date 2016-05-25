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
/*
 * This executable is an utility which allows to load an ontology defined in owl standard and
 * generate an header with URIs enumerated. The header is useful to avoid syntax mistakes in the code
 */

#include <dirent.h>
#include <ros/ros.h>
#include <stdio.h>
#include <boost/foreach.hpp>
#include "skiros_config/param_types.h"
#include "skiros_common/utility.h"
#include "skiros_world_model/owl_world_ontology.h"
#include "skiros_world_model/world_element.h"
#include "skiros_world_model/reasoners_loading_func.h"
#include "skiros_world_model/owl_world_model.h"

using namespace skiros_wm;

bool isFirstCharNumber(std::string input)
{
    try
    {
        boost::lexical_cast<int>(input.at(0));
        return true;
    }
    catch(boost::bad_lexical_cast e)
    {
        return false;
    }
}

/////////////////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////////////////

int main (int argc, char **argv)
{
	//Foundamental initialization necessary to avoid problems with serialization of params (see "skiros_config/param_types.h" for more details)
	skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();

    skiros_wm::owl::Ontology ontology;
    skiros_wm::owl::WorldModel model(ontology);

    ros::Time::init();

    while(true)
	{
	    char c = 'n';
        std::cout << "Char: 'l' load a different world model. 'g' generate the header. 'c' close." << std::endl;
	    std::cin >> c;
	    if (c == 'l') //Load a file
	    {
            struct dirent *dirpent;
            DIR * dirp;
            dirp = opendir(ontology.getWorkspacePath().c_str());
            std::cout << ontology.getWorkspacePath() << ": " << std::endl;

            if(dirp)
            {
                std::vector<std::string> v;int i = 0;
                //List the .xml files found in the directory
                while((dirpent=readdir(dirp)) != NULL)
                {
                  std::string temp(dirpent->d_name);
                  if(temp.find(".owl") != std::string::npos)
                  {
                  std::cout << i++ << ": " << dirpent->d_name << std::endl;
                  v.push_back(temp);
                  }
                }
                closedir(dirp);
                //Wait for a selection
                while(true)
                {
                  if(v.size()>0)
                  {
                std::cout << "Select the file to load. (number)" << std::endl;
                std::cin >> i;
                if(i>=0 && i < v.size())
                {
                    try{
                    model.loadMainOntology(ontology.getWorkspacePath()+v[i]);
                    std::cout << "File loaded successfully." << std::endl;
                    }
                    catch(std::runtime_error e){FERROR(e.what());}
                    break;
                }
                else
                {
                  std::cout << "Not valid input. Try again." << std::endl;
                  std::cin.clear();
                  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                }
                  }
                  else
                  {
                  std::cout << "No files to load." << std::endl;
                  break;
                  }
                }
		}
		else  std::cout << "Problem occured while opening the directory." << std::endl;
        } else if (c == 'g')
        {
            std::string filename = ros::package::getPath("skiros_config")+"/include/skiros_config/"+"declared_uri.h";
            FILE* f = std::fopen(filename.c_str(), "w");
            std::string str = "#ifndef DECLARED_URI_H \n#define DECLARED_URI_H \nnamespace skiros_config \n{ \n";
            fprintf(f, "%s", str.c_str());

            //--------------------------------------- CONCEPTS --------------------------------------------
            fprintf(f, "%s", "namespace owl \n{\n");
            fprintf(f, "%s", "namespace concept \n{\n");
            fprintf(f, "%s", "enum ConceptType \n{\n");
            std::set<std::string> class_set = model.getDeclaredClasses();
            BOOST_FOREACH(std::string concept, class_set)
            {
                if(isFirstCharNumber(concept)) fprintf(f, "    i%s, \n", concept.c_str());
                else fprintf(f, "    %s, \n", concept.c_str());
            }
            fprintf(f, "%s", "}; \n \nstatic const char * Str[] = \n{ \n");
            BOOST_FOREACH(std::string concept, class_set)
            {
                fprintf(f, "    \"%s\", \n", concept.c_str());
            }
            fprintf(f, "%s", "}; \n} \n");


            //--------------------------------------- RELATIONS --------------------------------------------
            fprintf(f, "%s", "namespace relation \n{\n");
            fprintf(f, "%s", "enum RelationType \n{\n");
            std::set<std::string> relation_set = model.getDeclaredRelations();
            BOOST_FOREACH(std::string relation, relation_set)
            {
                if(isFirstCharNumber(relation)) fprintf(f, "    i%s, \n", relation.c_str());
                else fprintf(f, "    %s, \n", relation.c_str());
            }
            fprintf(f, "%s", "}; \n \nstatic const char * Str[] = \n{ \n");
            BOOST_FOREACH(std::string relation, relation_set)
            {
                fprintf(f, "    \"%s\", \n", relation.c_str());
            }
            fprintf(f, "%s", "}; \n} \n");



            //--------------------------------------- Data --------------------------------------------
            fprintf(f, "%s", "namespace data \n{\n");
            fprintf(f, "%s", "enum DataType \n{\n");
            std::set<std::string> data_set = model.getDeclaredData();
            BOOST_FOREACH(std::string data, data_set)
            {
                if(isFirstCharNumber(data)) fprintf(f, "    i%s, \n", data.c_str());
                else fprintf(f, "    %s, \n", data.c_str());
            }
            fprintf(f, "%s", "}; \n \nstatic const char * Str[] = \n{ \n");
            BOOST_FOREACH(std::string data, data_set)
            {
                fprintf(f, "    \"%s\", \n", data.c_str());
            }
            fprintf(f, "%s", "}; \n} \n");


            //--------------------------------------- INDIVIDUALS --------------------------------------------

            fprintf(f, "%s", "namespace individual \n{\n");
            fprintf(f, "%s", "enum IndividualType \n{\n");
            std::set<std::string> ind_map = model.getDeclaredIndividuals();
            BOOST_FOREACH(std::string pair, ind_map)
            {
                if(pair.find("-")!=std::string::npos)continue;//Skip any eventual world model instance (characterized by having the '-' in the name)
                if(isFirstCharNumber(pair)) fprintf(f, "    i%s, \n", pair.c_str());
                else fprintf(f, "    %s, \n", pair.c_str());
            }
            fprintf(f, "%s", "}; \n \nstatic const char * Str[] = \n{ \n");
            BOOST_FOREACH(std::string pair, ind_map)
            {
                if(pair.find("-")!=std::string::npos)continue;//Skip any eventual world model instance (characterized by having the '-' in the name)
                fprintf(f, "    \"%s\", \n", pair.c_str());
            }
            str = "}; \n \n}\n} }\n\n#endif //DECLARED_URI_H";
            fprintf(f, "%s", str.c_str());
            fclose(f);
            FINFO("File generated: " << filename);
        }
        else if (c == 'c') break;
	}
	return 0;
} // main

