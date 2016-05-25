#ifndef BaseArmDevice_HPP
#define BaseArmDevice_HPP

#include <ros/ros.h>
#include <skiros_resource/base_device.h>
#include <string>

namespace skiros_resource
{

		/*! \brief Define the generic template for an arm proxy.
		 *
		 *  Detailed description: TODO
		 */
		class BaseArmDevice : public BaseDevice
		{
			public:


				virtual ~BaseArmDevice() {}

                virtual void setModality(std::string) = 0;

                virtual void setPayload(double) = 0;

                //virtual void move_ptp(){}

                //virtual void move_lin(){}


			protected:
				BaseArmDevice() : BaseDevice() {}

		};

}


#endif // BaseArmDevice_HPP
