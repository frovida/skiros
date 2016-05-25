#ifndef BASEDEVICE_HPP
#define BASEDEVICE_HPP

#include <ros/ros.h>
#include <string>


namespace skiros_resource
{

        struct DriverType
        {
            std::string id_name; //ID
            std::string package;
            std::string executable;
            std::string parameters;
            //If ROS node
            std::string state;
            int pid;
        };


        /*!
         * \brief Define the generic template for a device.
         *
		 */
        class BaseDevice
		{
			public:

                inline void name(std::string name){this->name_ = name;}
                inline std::string name() const { return name_; }

                inline void brand(std::string brand){this->brand_ = brand;}
                inline std::string brand()  { return brand_; }

                inline void model(std::string model){this->model_ = model;}
                inline std::string model() const { return model_; }

                inline void package(std::string package){this->package_ = package;}
                inline std::string package() const { return package_; }

                inline void node(std::string node){this->node_ = node;}
                inline std::string node() const { return node_; }

                BaseDevice() : state_msg_("") {}
                virtual ~BaseDevice() {}

				void setStateMsg(std::string state_msg)
				{
					state_msg_ = state_msg;
				}

				virtual std::string getSpecificationDescriptions(void) const
				{
					return "Specification Description not specified for this Device.";
				}

                virtual DriverType getRelatedDriver(void) const
				{
                    return DriverType();
				}

            private:
				std::string state_msg_;
				std::string related_driver_;

			    //device data variables:
                std::string name_;
                std::string brand_;
                std::string model_;
                std::string package_;
                std::string node_;
		};

}

#endif // BASEDEVICE_HPP
