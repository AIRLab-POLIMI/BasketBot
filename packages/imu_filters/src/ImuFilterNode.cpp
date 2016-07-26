#include <Module.hpp>

#include <imu_filters/ImuFilterNode.hpp>
#include <Core/Utils/Math/Constants.hpp>
#include <Core/Utils/Math/Conversions.hpp>
#include <L3GD20H_driver/L3GD20H.hpp>
#include <LSM303D_driver/LSM303D.hpp>


namespace imu_filters {
   ImuFilterNode::ImuFilterNode(
      const char* name,
	  Filter& filter,
      Core::MW::Thread::Priority priority
   ) :
      CoreNode::CoreNode(name, priority), filter(filter)
   {
      _workingAreaSize = 512;
   }

   ImuFilterNode::~ImuFilterNode()
   {
      teardown();
   }

   bool
   ImuFilterNode::onConfigure()
   {
	   auto& c = configuration;
	   filter.config(1.0f/c.frequency);
	   _deltaT = Core::MW::Time::ms(1000.0f/c.frequency);
	   _stamp = Core::MW::Time::now();

	   return true;
   }

   bool
   ImuFilterNode::onPrepareMW()
   {
      this->advertise(_publisher, configuration.topic);

      return true;
   }

   bool
   ImuFilterNode::onLoop()
   {
	   Core::MW::Thread::sleep_until(_stamp+_deltaT);
	   Module::acc.get(_accData);
	   Module::gyro.get(_gyroData);
	   Module::mag.get(_magData);

	   adjustMeasurements();
	   filter(measure);

	   sensor_msgs::Imu_f32* msgp;
	   if(_publisher.alloc(msgp))
	   {
		   msgp->orientation = filter.attitude;
		   msgp->linear_acceleration = filter.linear_acceleration;
		   msgp->angular_velocity = filter.angular_velocity;
		   _publisher.publish(msgp);
	   }
	   _stamp = Core::MW::Time::now();

       return true;
   } // Mahony::onLoop


   void ImuFilterNode::adjustMeasurements()
   {
	   auto& acc_a = configuration.acc_a;
	   auto& acc_b = configuration.acc_b;

	   measure.acc[0] = acc_a[0]*_accData.x + acc_b[0];
	   measure.acc[1] = acc_a[1]*_accData.y + acc_b[1];
	   measure.acc[2] = acc_a[2]*_accData.z + acc_b[2];

	   auto& gyr_a = configuration.gyr_a;
	   auto& gyr_b = configuration.gyr_b;

	   measure.gyr[0] = gyr_a[0]*_gyroData.y    + gyr_b[0];
	   measure.gyr[1] = gyr_a[1]*(-_gyroData.x) + gyr_b[1];
	   measure.gyr[2] = gyr_a[2]*_gyroData.z    + gyr_b[2];

	   auto& mag_a = configuration.mag_a;
	   auto& mag_b = configuration.mag_b;

	   measure.mag[0] = mag_a[0]*_magData.x + mag_b[0];
	   measure.mag[1] = mag_a[1]*_magData.y + mag_b[1];
	   measure.mag[2] = mag_a[2]*_magData.z + mag_b[2];
   }

}
 
