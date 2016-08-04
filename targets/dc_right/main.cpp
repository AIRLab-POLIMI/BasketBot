//-----------------//
// DC right module //
//-----------------//

#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/sensor_publisher/Publisher.hpp>
#include <core/actuator_subscriber/Subscriber.hpp>
#include <core/led/Subscriber.hpp>

// --- BOARD IMPL -------------------------------------------------------------
#include <core/QEI_driver/QEI.hpp>
#include <core/A4957_driver/A4957.hpp>
#include <core/current_control/CurrentPID.hpp>
#include <core/current_control/Calibration.hpp>

// *** DO NOT MOVE ***
Module module;

// --- TYPES ------------------------------------------------------------------
using QEI_Publisher = core::sensor_publisher::Publisher<ModuleConfiguration::QEI_DELTA_DATATYPE>;
using CurrentSensor = core::current_control::CurrentSensor;
using CurrentPID = core::current_control::CurrentPID;
using Calibration = core::current_control::Calibration;

// --- NODES ------------------------------------------------------------------

//#define CALIBRATION
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);

#ifdef CALIBRATION
Calibration calibration("calibration", module.hbridge_pwm, core::os::Thread::PriorityEnum::NORMAL);
#else
QEI_Publisher encoder("encoder", module.qei, core::os::Thread::PriorityEnum::NORMAL);
CurrentSensor currentSensor; //TODO move in module
CurrentPID currentPid("current_pid", currentSensor, module.hbridge_pwm, core::os::Thread::PriorityEnum::NORMAL);
#endif

// --- MAIN -------------------------------------------------------------------
extern "C" {
   int
   main()
   {
      module.initialize();

      currentSensor.init(); //TODO move in module

      // Module configuration
      core::QEI_driver::QEI_DeltaConfiguration qei_conf;
      qei_conf.period = 50;
      qei_conf.ticks = 1000;
      module.qei.setConfiguration(qei_conf);

      // Nodes configuration
      core::led::SubscriberConfiguration led_conf;
      led_conf.topic = "led";
      led_subscriber.setConfiguration(led_conf);

      #ifndef CALIBRATION
      core::sensor_publisher::Configuration encoder_conf;
      encoder_conf.topic = "encoder_right";
      encoder.setConfiguration(encoder_conf);

      currentSensor.configuration.a = 0.007432946790511f;
      currentSensor.configuration.b = -15.107809133385506f;

      currentPid.configuration.maxV = 24;
      currentPid.configuration.R = 0.299f;
      currentPid.configuration.L = 8.2e-5f;
      currentPid.configuration.T = 0.0115f;
      currentPid.configuration.Kt = 30.2e-3;

      currentPid.configuration.controlCycles = 1;
      currentPid.configuration.omegaC = 6000.0f;
      currentPid.configuration.topic = "torque_right";
#endif

      // Add nodes to the node manager (== board)...
      module.add(led_subscriber);

#ifdef CALIBRATION
      module.add(calibration);
#else
      module.add(encoder);
      module.add(currentPid);
#endif

      // ... and let's play!
      module.setup();
      module.run();

      // Is everything going well?
      for (;;) {
         if (!module.isOk()) {
            module.halt("This must not happen!");
         }

         core::os::Thread::sleep(core::os::Time::ms(500));
      }

      return core::os::Thread::OK;
   } // main
}
