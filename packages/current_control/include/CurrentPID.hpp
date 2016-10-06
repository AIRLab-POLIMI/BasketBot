#pragma once

#include <core/mw/CoreNode.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/utils/BasicActuator.hpp>

#include <core/current_control/CurrentSensor.hpp>
#include <core/current_control/CurrentPIDConfiguration.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>
#include <core/pid_ie/pid_ie.hpp>

namespace core
{
namespace current_control {
   class CurrentPID:
      public core::mw::CoreNode,
	  public core::mw::CoreConfigurable<core::current_control::CurrentPIDConfiguration>
   {
public:
      CurrentPID(
         const char* name,
		 CurrentSensor& currentSensor,
		 core::utils::BasicActuator<float>& pwm,
         core::os::Thread::PriorityEnum priority = core::os::Thread::PriorityEnum::NORMAL
      );

      virtual
      ~CurrentPID();

      void controlCallback(float currentPeak, float Vcc);

private:
      CurrentSensor& _currentSensor;
      core::utils::BasicActuator<float>& _pwm;
      core::mw::Subscriber<actuator_msgs::Setpoint_f32, 5> _subscriber;

      pid_ie::PID_IE _currentPID;

      int _sign;
      float _Kpwm;
      float _Ktorque;
      float _current;
      uint32_t _controlCounter;
      uint32_t _controlCycles;

      bool _underVoltage;


private:
      bool
      onConfigure();
    
      bool
      onPrepareHW();
    
      bool
      onPrepareMW();

      bool
	  onStart();

      bool
      onLoop();

      bool
	  onStop();

      static bool callback(const actuator_msgs::Setpoint_f32& msg,
         core::mw::Node* node);

      bool checkDriverState(float Vcc);
   };
} 
}
