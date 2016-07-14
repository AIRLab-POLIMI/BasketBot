#include <Module.hpp>

#include <mahony/Mahony.hpp>
#include <Core/Utils/Math/Constants.hpp>
#include <Core/Utils/Math/Conversions.hpp>
#include <L3GD20H_driver/L3GD20H.hpp>
#include <LSM303D_driver/LSM303D.hpp>


namespace mahony {
   Mahony::Mahony(
      const char*                name,
      Core::MW::Thread::Priority priority
   ) :
      CoreNode::CoreNode(name, priority)
   {
      _workingAreaSize = 512;
   }

   Mahony::~Mahony()
   {
      teardown();
   }

   bool
   Mahony::onConfigure()
   {
	   auto& c = configuration;
	   filter.config(c.Kp,c.Ki, c.Kacc, c.Kmag, 1.0f/c.frequency);
	   _deltaT = Core::MW::Time::ms(1000.0f/c.frequency);
	   _stamp = Core::MW::Time::now();

	   return true;
   }

   bool
   Mahony::onPrepareMW()
   {
      _subscriberGyro.set_callback(Mahony::gyroCallback);
      _subscriberAcc.set_callback(Mahony::accCallback);
      _subscriberMag.set_callback(Mahony::magCallback);

      this->subscribe(_subscriberGyro, configuration.topicGyro);
      this->subscribe(_subscriberAcc, configuration.topicAcc);
      this->subscribe(_subscriberMag, configuration.topicMag);

      this->advertise(_publisher, configuration.topic);

      return true;
   }

   bool
   Mahony::onLoop()
   {
	   Core::MW::Thread::sleep_until(_stamp+_deltaT);
	   adjustMeasurements();
	   filter(measure);
	   _publisher.publish(filter.attitude);
	   _stamp = Core::MW::Time::now();

       return true;
   } // Mahony::onLoop

   bool
   Mahony::gyroCallback(
      const Configuration::L3GD20H_GYRO_DATATYPE& msg,
      Core::MW::Node*                             node
   )
   {
      Mahony* tmp = static_cast<Mahony*>(node);

      tmp->_gyroData = msg;

      return true;
   }

   bool
   Mahony::accCallback(
      const Configuration::LSM303D_ACC_DATATYPE& msg,
      Node*                                      node
   )
   {
      Mahony* tmp = static_cast<Mahony*>(node);

      tmp->_accData = msg;

      return true;
   }

   bool
   Mahony::magCallback(
      const Configuration::LSM303D_MAG_DATATYPE& msg,
      Node*                                      node
   )
   {
      Mahony* tmp = static_cast<Mahony*>(node);

      tmp->_magData = msg;

      return true;
   }

   void Mahony::adjustMeasurements()
   {
	   auto& acc_a = configuration.acc_a;
	   auto& acc_b = configuration.acc_a;

	   measure.acc[0] = acc_a[0]*_accData.x + acc_b[0];
	   measure.acc[1] = acc_a[1]*_accData.y + acc_b[1];
	   measure.acc[2] = acc_a[2]*_accData.z + acc_b[2];

	   auto& gyr_a = configuration.gyr_a;
	   auto& gyr_b = configuration.gyr_a;

	   measure.gyr[0] = gyr_a[0]*_gyroData.y    + gyr_b[0];
	   measure.gyr[1] = gyr_a[1]*(-_gyroData.x) + gyr_b[1];
	   measure.gyr[2] = gyr_a[2]*_gyroData.z    + gyr_b[2];

	   auto& mag_a = configuration.mag_a;
	   auto& mag_b = configuration.mag_a;

	   measure.mag[0] = mag_a[0]*_magData.x + mag_b[0];
	   measure.mag[1] = mag_a[1]*_magData.y + mag_b[1];
	   measure.mag[2] = mag_a[2]*_magData.z + mag_b[2];
   }

}
 
