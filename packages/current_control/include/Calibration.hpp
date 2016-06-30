#pragma once

#include <Core/MW/CoreNode.hpp>
#include <Core/MW/Subscriber.hpp>
#include <Core/MW/CoreActuator.hpp>

#include <current_control/CurrentPIDConfiguration.hpp>
#include <actuator_msgs/Setpoint_f32.hpp>
#include <pid_ie/pid_ie.hpp>

namespace current_control {
   class Calibration:
      public Core::MW::CoreNode
   {
public:
	   Calibration(
         const char*                    name,
		 Core::MW::CoreActuator<float>& pwm,
         Core::MW::Thread::PriorityEnum priority = Core::MW::Thread::PriorityEnum::NORMAL
      );

      virtual
      ~Calibration();

      void calibrationCallback();

private:
      Core::MW::CoreActuator<float>& _pwm;

      float _current;


private:
      bool
      onPrepareHW();

      bool
      onLoop();

      static bool callback(const actuator_msgs::Setpoint_f32& msg,
         Core::MW::Node* node);
   };
} 