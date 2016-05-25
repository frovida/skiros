#ifndef BaseCameraDevice_HPP
#define BaseCameraDevice_HPP

#include <ros/ros.h>
#include <skiros_resource/base_device.h>
#include <string>

namespace skiros_resource
{

        /*! \brief Define the generic template for an arm proxy.
         *
         *  Detailed description: TODO
         */
        class BaseCameraDevice : public BaseDevice
        {
            public:

                virtual ~BaseCameraDevice() {}

                virtual void toggle() = 0;

            protected:
                BaseCameraDevice() : BaseDevice() {}

        };

}


#endif // BaseCameraDevice_HPP
