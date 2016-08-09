#pragma once

#include <core/mw/CoreNode.hpp>
#include <core/mw/Publisher.hpp>

#include <core/current_control/BroadcasterConfiguration.hpp>
#include <core/current_control/CurrentSensor.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

namespace core
{
namespace current_control {
   class Broadcaster:
      public core::mw::CoreNode,
	  public core::mw::CoreConfigurable<core::current_control::BroadcasterConfiguration>
   {

   public:
	   Broadcaster(
            const char* name,
   		 CurrentSensor& currentSensor,
            core::os::Thread::PriorityEnum priority = core::os::Thread::PriorityEnum::NORMAL
         );

         virtual
         ~Broadcaster();

   private:
         core::mw::Publisher<core::actuator_msgs::Setpoint_f32> _publisher;
         CurrentSensor& _currentSensor;
         float _current;

         core::os::Time _deltaT;
         core::os::Time _stamp;

   private:
         bool
         onConfigure();

         bool
         onPrepareMW();

         bool
         onLoop();

   };

}
}
