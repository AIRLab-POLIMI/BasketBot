#pragma once

#include <core/mw/CoreNode.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreActuator.hpp>

#include <core/current_control/CalibrationConfiguration.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>
#include <core/pid_ie/pid_ie.hpp>

namespace core {
namespace current_control {
   class Calibration:
      public core::mw::CoreNode,
	  public core::mw::CoreConfigurable<core::current_control::CalibrationConfiguration>
   {
public:
	   Calibration(
         const char*                    name,
		 core::mw::CoreActuator<float>& pwm,
		 bool positive,
         core::os::Thread::PriorityEnum priority = core::os::Thread::PriorityEnum::NORMAL
      );

      virtual
      ~Calibration();

      void calibrationCallback(uint16_t current);

private:
      core::mw::CoreActuator<float>& _pwm;
      const bool _positive;

      uint16_t _current;


private:
      bool
      onPrepareHW();

      bool
      onLoop();

      static bool callback(const actuator_msgs::Setpoint_f32& msg,
         core::mw::Node* node);
   };
} 
}
