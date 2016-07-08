#include <Module.hpp>

#include <mahony/Mahony.hpp>
#include <Core/Utils/Math/Constants.hpp>
#include <Core/Utils/Math/Conversions.hpp>
#include <sensor_msgs/RPY_f32.hpp>
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
}
 
