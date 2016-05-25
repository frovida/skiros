#ifndef BaseGripperDevice_H
#define BaseGripperDevice_H

#include <string>
#include <vector>
#include <skiros_msgs/Finger.h>
#include <skiros_msgs/FingerFB.h>
#include <skiros_resource/base_device.h>


namespace skiros_resource
{
		/*
		 * A finger is defined as a rigid and flat surface that can apply a positive (impactive) or negative (astrictive) force
		 * on another surface
		 */
		/*
		static const int  ASTRICTIVE_FINGER 	= 0;
		static const int  IMPACTIVE_FINGER 		= 1;

		struct finger
		{
			//Area
			double width;
			double lenght;
			//Astrictive or impactive
			int type;
			//Astrictive can be static fingers
			bool actuated;
		};*/

		/*! \brief Define the generic template for a gripper proxy.
		 *
		 *  Detailed description: TODO
		 */
		class BaseGripperDevice : public BaseDevice
		{
		public:

			virtual ~BaseGripperDevice() = 0;

            virtual void init(ros::NodeHandle) = 0;

            virtual bool isReadyToMove(ros::Duration) = 0;

			virtual void reset() = 0;

			virtual void activate() = 0;

			virtual void fingersToPos(int, std::vector<skiros_msgs::Finger> ) = 0;

			virtual void open() = 0;

			virtual void close() = 0;

			virtual void stop() = 0;

			virtual bool changeModality(int) = 0;

			virtual void test() = 0;

            virtual void getFeedback(std::vector<skiros_msgs::FingerFB> &) = 0;

			virtual int getNumOfModalities() = 0;

            virtual int getCurrentModality() = 0;
			/*
			 * Used to retrieve the number of fingers actuated.
			 * The number can change, depending on the modality in use.
			 */

			virtual int getNumOfFingers() = 0;

			/*
			 * Calculates the manifold of the finger (just an idea for now)
			 */

			//virtual void getFingerManifold();

			/*
			 * Used to retrieve the finger definition (type and contact area).
			 * The definition can change depending on the modality in use.
			 */

			//virtual finger getFingerType(int);


            virtual DriverType getRelatedDriver(void) const
            {
                return DriverType();
            }

		protected:
			BaseGripperDevice() {}
		};

}



#endif // BaseGripperDevice_H
