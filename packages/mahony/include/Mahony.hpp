#pragma once

#include <Core/MW/Publisher.hpp>
#include <Core/MW/Subscriber.hpp>
#include <Core/MW/CoreNode.hpp>
#include <Core/MW/CoreSensor.hpp>
#include <Core/MW/Callback.hpp>

#include <Configuration.hpp>

#include <sensor_msgs/Imu_f32.hpp>
#include <mahony/MahonyConfiguration.hpp>
#include <mahony/Measurement.hpp>
#include <mahony/MahonyFilter.hpp>

namespace mahony {
   class Mahony:
      public Core::MW::CoreNode
   {
public:
      Mahony(
         const char*                name,
         Core::MW::Thread::Priority priority = Core::MW::Thread::PriorityEnum::NORMAL
      );
      virtual
      ~Mahony();

public:
      MahonyConfiguration configuration;

private:
      Configuration::L3GD20H_GYRO_DATATYPE _gyroData;
      Configuration::LSM303D_ACC_DATATYPE  _accData;
      Configuration::LSM303D_MAG_DATATYPE  _magData;

      measurement measure;
      MahonyFilter filter;

private:
      bool
      onConfigure();

      bool
      onPrepareMW();

      bool
      onLoop();

private:
      void
	  adjustMeasurements();



private:
      Core::MW::Publisher<sensor_msgs::Imu_f32> _publisher;

      Core::MW::Time _deltaT;
      Core::MW::Time _stamp;
   };
} 
