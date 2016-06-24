#pragma once

#include <Core/MW/CoreNode.hpp>
#include <Core/MW/Subscriber.hpp>
#include <Core/MW/CoreActuator.hpp>

#include <current_control/CurrentPIDConfiguration.hpp>
#include <extra_msgs/Current2.hpp>

namespace current_control {
   class CurrentPID:
      public Core::MW::CoreNode
   {
public:
      CurrentPID(
         const char*                    name,
		 Core::MW::CoreActuator<float>& pwm,
         Core::MW::Thread::PriorityEnum priority = Core::MW::Thread::PriorityEnum::NORMAL
      );
      virtual
      ~CurrentPID();

public:
      CurrentPIDConfiguration configuration;

private:
      Core::MW::CoreActuator<float>& _pwm;
      Core::MW::Subscriber<extra_msgs::Current2, 5> _subscriber;

private:
      bool
      onConfigure();
    
      bool
      onPrepareHW();
    
      bool
      onPrepareMW();

      bool
      onLoop();
   };
} 
