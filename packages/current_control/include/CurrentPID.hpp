#pragma once

#include <Core/MW/CoreNode.hpp>
#include <Core/MW/Subscriber.hpp>
#include <Core/MW/CoreActuator.hpp>

#include <current_control/CurrentSensor.hpp>
#include <current_control/CurrentPIDConfiguration.hpp>
#include <actuator_msgs/Setpoint_f32.hpp>
#include <pid_ie/pid_ie.hpp>

namespace current_control {
   class CurrentPID:
      public Core::MW::CoreNode
   {
public:
      CurrentPID(
         const char* name,
		 CurrentSensor& currentSensor,
		 Core::MW::CoreActuator<float>& pwm,
         Core::MW::Thread::PriorityEnum priority = Core::MW::Thread::PriorityEnum::NORMAL
      );

      virtual
      ~CurrentPID();

      void controlCallback(float currentPeak);

public:
      CurrentPIDConfiguration configuration;

private:
      CurrentSensor& _currentSensor;
      Core::MW::CoreActuator<float>& _pwm;
      Core::MW::Subscriber<actuator_msgs::Setpoint_f32, 5> _subscriber;

      pid_ie::PID_IE _currentPID;

      float _Kpwm;
      float _current;
      uint32_t _controlCounter;
      uint32_t _controlCycles;


private:
      bool
      onConfigure();
    
      bool
      onPrepareHW();
    
      bool
      onPrepareMW();

      bool
      onLoop();

      static bool callback(const actuator_msgs::Setpoint_f32& msg,
         Core::MW::Node* node);
   };
} 
