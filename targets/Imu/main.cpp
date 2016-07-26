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
#include <imu_filters/ImuFilterNode.hpp>
#include <imu_filters/MahonyFilter.hpp>

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

imu_filters::MahonyFilter mahony_filter;
imu_filters::ImuFilterNode imu_filter("imu_filter", mahony_filter);

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

		// Led publisher
		led_publisher.configuration.topic = "led";
		led_publisher.configuration.led = (uint32_t)1;
		module.add(led_publisher);

		// Sensor nodes
		gyro_publisher.configuration.topic = "gyro";
		acc_publisher.configuration.topic  = "acc";
		mag_publisher.configuration.topic  = "mag";
		module.add(gyro_publisher);
		module.add(acc_publisher);
		module.add(mag_publisher);

		// Mahony filter node
		imu_filter.configuration.topic     = "imu";
		imu_filter.configuration.frequency = 100.0f;

		//Imu calibration parameters
		imu_filter.configuration.acc_a = { 0.000982402074221, 0.000947431355433, 0.001007804098727};
		imu_filter.configuration.acc_b = {0.010529343232548, 0.044172464327909, -0.113292069793833};
		imu_filter.configuration.gyr_a = {0.311704244531320e-3, 0.314591558752256e-3, 0.3066036023887353e-3};
		imu_filter.configuration.gyr_b = {-0.148446667051465, -0.012753980951780, 0.025803887886194};
		imu_filter.configuration.mag_a = {0.002166223718555, 0.002341707054921, 0.002613937090468};
		imu_filter.configuration.mag_b = {-0.160726372652952, 0.093705049503890, 0.077222493617249};

		// mahony filter parameters
		mahony_filter.configuration.Kacc = 4.0f;
		mahony_filter.configuration.Kmag = 0.1f;
		mahony_filter.configuration.Kp   = 0.5f;
		mahony_filter.configuration.Ki   = 0.1f;

		module.add(imu_filter);

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
