#include <Configuration.hpp>
#include <Module.hpp>

// MESSAGES
#include <common_msgs/Led.hpp>
#include <common_msgs/String64.hpp>
#include <sensor_msgs/RPY_f32.hpp>

// NODES
#include <sensor_publisher/Publisher.hpp>
#include <led/Publisher.hpp>
#include <led/Subscriber.hpp>
#include <mahony/Mahony.hpp>

// BOARD IMPL
#include <L3GD20H_driver/L3GD20H.hpp>
#include <LSM303D_driver/LSM303D.hpp>

// *** DO NOT MOVE ***
Module module;

// TYPES
using Vector3_i16_Publisher = sensor_publisher::Publisher<common_msgs::Vector3_i16>;

// NODES
Vector3_i16_Publisher gyro_publisher("gyro_publisher", module.gyro, Core::MW::Thread::PriorityEnum::NORMAL + 1);
Vector3_i16_Publisher acc_publisher("acc_publisher", module.acc, Core::MW::Thread::PriorityEnum::NORMAL + 1);
Vector3_i16_Publisher mag_publisher("mag_publisher", module.mag, Core::MW::Thread::PriorityEnum::NORMAL + 1);

led::Publisher led_publisher("led_publisher", Core::MW::Thread::PriorityEnum::LOWEST);
led::Subscriber led_subscriber("led_subscriber", Core::MW::Thread::PriorityEnum::LOWEST);
//madgwick::Madgwick   madgwick_filter("madgwick");
mahony::Mahony   mahony_filter("madgwick");

/*===========================================================================*/
/* Kinematics.                                                               */
/*===========================================================================*/
/*
 *               y
 *               ^
 *               |
 *               |
 *   2           @---->x     1
 *  ||                       ||  |
 *  ||_______________________||  | R
 *  ||                       ||
 *  ||                       ||
 *               L
 *   <----------------------->
 *
 */

// Robot parameters
#define basketbot_L        0.220f     // Wheel distance [m]
#define basketbot_R        (0.155f / 2) // Wheel radius [m]


// MAIN
extern "C" {
	int
	main()
	{
		module.initialize();

		// Led subscriber node
		led_subscriber.configuration.topic = "led";
		module.add(led_subscriber);

		// Sensor nodes
		gyro_publisher.configuration.topic = "gyro";
		acc_publisher.configuration.topic  = "acc";
		mag_publisher.configuration.topic  = "mag";
		module.add(gyro_publisher);
		module.add(acc_publisher);
		module.add(mag_publisher);

		// Madgwick filter node
		mahony_filter.configuration.topicGyro = gyro_publisher.configuration.topic;
		mahony_filter.configuration.topicAcc  = acc_publisher.configuration.topic;
		mahony_filter.configuration.topicMag  = mag_publisher.configuration.topic;
		mahony_filter.configuration.topic     = "imu";
		mahony_filter.configuration.frequency = 50.0f;
		module.add(mahony_filter);

		// Setup and run
		module.setup();
		module.run();

		// Is everything going well?
		for (;;) {
			if (!module.isOk()) {
				module.halt("This must not happen!");
			}

			Core::MW::Thread::sleep(Core::MW::Time::ms(500));
		}

		return Core::MW::Thread::OK;
	}
}
