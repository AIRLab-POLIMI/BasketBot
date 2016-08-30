//----------------//
// DC left module //
//----------------//

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
#include <core/current_control/Broadcaster.hpp>

// *** DO NOT MOVE ***
Module module;

// --- TYPES ------------------------------------------------------------------
using QEI_Publisher = core::sensor_publisher::Publisher<ModuleConfiguration::QEI_DELTA_DATATYPE>;
using CurrentSensor = core::current_control::CurrentSensor;
using CurrentPID = core::current_control::CurrentPID;
using Subscriber = core::actuator_subscriber::Subscriber<float, core::actuator_msgs::Setpoint_f32>;
using Broadcaster = core::current_control::Broadcaster;

// --- NODES ------------------------------------------------------------------

//#define CALIBRATION
CurrentSensor currentSensor; //TODO move in module
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);
Broadcaster broadcaster("broadcaster", currentSensor, core::os::Thread::PriorityEnum::LOWEST);

#ifdef CALIBRATION
Subscriber calibration ("calibration", module.hbridge_pwm, core::os::Thread::PriorityEnum::NORMAL);
#else
QEI_Publisher encoder("encoder", module.qei, core::os::Thread::PriorityEnum::NORMAL);
CurrentPID currentPid("current_pid", currentSensor, module.hbridge_pwm, core::os::Thread::PriorityEnum::NORMAL);
#endif

// --- CONFIGURATIONS ---------------------------------------------------------
core::QEI_driver::QEI_DeltaConfiguration qei_conf;
core::led::SubscriberConfiguration led_conf;
core::A4957_driver::A4957_SignMagnitudeConfiguration pwm_conf;
core::current_control::BroadcasterConfiguration broadcaster_conf;

#ifdef CALIBRATION
core::actuator_subscriber::Configuration calibration_conf;
#else
core::sensor_publisher::Configuration encoder_conf;
core::current_control::CurrentPIDConfiguration currentPid_conf;
#endif

// --- MAIN -------------------------------------------------------------------
extern "C" {
   int
   main()
   {
      // Module configuration
      const float encoderTicks = 4*500;
      const float transmissionRatio = 26.0*10.0/3.0;

      qei_conf.period = 10;
      qei_conf.ticks = encoderTicks*transmissionRatio;
      qei_conf.invert = 1;
      module.qei.setConfiguration(qei_conf);

      pwm_conf.kappa = 1.0;
      pwm_conf.frequency = 72000000;
      pwm_conf.period = 4096/2;
      module.hbridge_pwm.setConfiguration(pwm_conf);

      module.initialize();

      // Nodes configuration
      led_conf.topic = "led";
      led_subscriber.setConfiguration(led_conf);

#ifdef CALIBRATION
      //current sensor configuration
      currentSensor.configuration.a = 1.0;
      currentSensor.configuration.b = 0.0;

      //calibration configuration
      calibration_conf.topic = "cmd_vel";
      calibration.setConfiguration(calibration_conf);
#else
      //encoder configuration
      encoder_conf.topic = "encoder_left";
      encoder.setConfiguration(encoder_conf);

      //current sensor configuration
      currentSensor.configuration.a = 0.00735450542782f;
      currentSensor.configuration.b = -15.397505657501757f;

      //current Pid configuration
      currentPid_conf.maxV = 24;
      currentPid_conf.Voff = 5.0;
      currentPid_conf.R = 0.299f;
      currentPid_conf.L = 8.2e-5f;
      currentPid_conf.T = 0.0115f;
      currentPid_conf.Kt = 30.2e-3;

      currentPid_conf.controlCycles = 1;
      currentPid_conf.invert = 1;
      currentPid_conf.omegaC = 6000.0f;
      currentPid_conf.topic = "torque_left";
      currentPid.setConfiguration(currentPid_conf);

#endif

      //broadcaster configuration
      broadcaster_conf.frequency = 100;
      broadcaster_conf.topic = "current_left";

      broadcaster.setConfiguration(broadcaster_conf);


      // Add nodes to the node manager (== board)...
      module.add(led_subscriber);

      module.add(broadcaster);

#ifdef CALIBRATION
      module.add(calibration);
      currentSensor.start();
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
