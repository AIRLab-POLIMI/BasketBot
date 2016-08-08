#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/led/Subscriber.hpp>
#include <core/led/Publisher.hpp>
#include "rosserial.hpp"

// --- BOARD IMPL -------------------------------------------------------------

// --- MISC -------------------------------------------------------------------

// *** DO NOT MOVE ***
Module module;

// --- NODES ------------------------------------------------------------------
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);

rosserial::RosSerialPublisher rosserial_publisher("rosserial_publisher", core::os::Thread::PriorityEnum::NORMAL);

// --- CONFIGURATIONS ---------------------------------------------------------
core::led::SubscriberConfiguration led_conf;

/*
 * Application entry point.
 */
extern "C" {
	int	main(void)
	{
		module.initialize();

		// Led subscriber configuration
		led_conf.topic = "led";
		led_subscriber.setConfiguration(led_conf);

		// Add nodes to the node manager (== board)...
		module.add(led_subscriber);

		module.add(rosserial_publisher);

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
