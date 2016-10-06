#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/os/Callback.hpp>

#include <ModuleConfiguration.hpp>

#include <core/sensor_msgs/Imu.hpp>
#include <core/imu_filters/ImuFilterNodeConfiguration.hpp>
#include <core/imu_filters/Measurement.hpp>
#include <core/imu_filters/Filter.hpp>

namespace core
{
namespace imu_filters {
   class ImuFilterNode:
      public core::mw::CoreNode,
	  public core::mw::CoreConfigurable<core::imu_filters::ImuFilterNodeConfiguration>
   {
public:
	   ImuFilterNode(
         const char* name,
		 Filter& filter,
         core::os::Thread::Priority priority = core::os::Thread::PriorityEnum::NORMAL
      );
      virtual
      ~ImuFilterNode();

private:
      ModuleConfiguration::L3GD20H_GYRO_DATATYPE _gyroData;
      ModuleConfiguration::LSM303D_ACC_DATATYPE  _accData;
      ModuleConfiguration::LSM303D_MAG_DATATYPE  _magData;

      measurement measure;
      Filter& filter;

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
      core::mw::Publisher<sensor_msgs::Imu> _publisher;

      core::os::Time _deltaT;
      core::os::Time _stamp;
   };
} 

}
