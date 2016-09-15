//------------//
// IMU module //
//------------//

#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// MESSAGES
#include <core/common_msgs/Led.hpp>
#include <core/common_msgs/String64.hpp>
#include <core/sensor_msgs/RPY_f32.hpp>

// NODES
#include <core/sensor_publisher/Publisher.hpp>
#include <core/led/Publisher.hpp>
#include <core/led/Subscriber.hpp>
#include <core/imu_filters/ImuFilterNode.hpp>
#include <core/imu_filters/MahonyFilter.hpp>
#include <core/balancing_robot_control/ControlNode.hpp>

// BOARD IMPL
#include <core/L3GD20H_driver/L3GD20H.hpp>
#include <core/LSM303D_driver/LSM303D.hpp>

// *** DO NOT MOVE ***
Module module;

// TYPES
using Vector3_i16_Publisher = core::sensor_publisher::Publisher<core::common_msgs::Vector3_i16>;

// NODES
Vector3_i16_Publisher gyro_publisher("gyro_publisher", module.gyro, core::os::Thread::PriorityEnum::NORMAL + 1);
Vector3_i16_Publisher acc_publisher("acc_publisher", module.acc, core::os::Thread::PriorityEnum::NORMAL + 1);
Vector3_i16_Publisher mag_publisher("mag_publisher", module.mag, core::os::Thread::PriorityEnum::NORMAL + 1);

core::led::Publisher led_publisher("led_publisher", core::os::Thread::PriorityEnum::LOWEST);
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);

core::imu_filters::MahonyFilter mahony_filter("mahony_filter");
core::imu_filters::ImuFilterNode imu_filter("imu_filter", mahony_filter);

core::balancing_robot_control::ControlNode control_node("controller", core::os::Thread::PriorityEnum::LOWEST);

// CONFIGURATIONS
core::sensor_publisher::Configuration gyro_conf;
core::sensor_publisher::Configuration acc_conf;
core::sensor_publisher::Configuration mag_conf;

core::led::SubscriberConfiguration led_subscriber_conf;
core::led::PublisherConfiguration led_publisher_conf;

core::imu_filters::ImuFilterNodeConfiguration imu_filter_conf;
core::imu_filters::MahonyConfiguration mahony_conf;

core::balancing_robot_control::ControlNodeConfiguration control_node_conf;


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
		led_subscriber_conf.topic = "led";
		led_subscriber.setConfiguration(led_subscriber_conf);
		module.add(led_subscriber);

		// Led publisher
		led_publisher_conf.topic = "led";
		led_publisher_conf.led = (uint32_t)1;
		led_publisher.setConfiguration(led_publisher_conf);
		module.add(led_publisher);

		// Sensor nodes
		gyro_conf.topic = "gyro";
		acc_conf.topic  = "acc";
		mag_conf.topic  = "mag";

		gyro_publisher.setConfiguration(gyro_conf);
		acc_publisher.setConfiguration(acc_conf);
		mag_publisher.setConfiguration(mag_conf);

		module.add(gyro_publisher);
		module.add(acc_publisher);
		module.add(mag_publisher);

		// Imu filter node
		imu_filter_conf.topic     = "imu";
		imu_filter_conf.frequency = 100.0f;

		imu_filter_conf.acc_a = { 0.000982402074221, 0.000947431355433, 0.001007804098727};
		imu_filter_conf.acc_b = {0.010529343232548, 0.044172464327909, -0.113292069793833};
		imu_filter_conf.gyr_a = {0.311704244531320e-3, 0.314591558752256e-3, 0.3066036023887353e-3};
		imu_filter_conf.gyr_b = {-0.148446667051465, -0.012753980951780, 0.025803887886194};
		imu_filter_conf.mag_a = {0.002166223718555, 0.002341707054921, 0.002613937090468};
		imu_filter_conf.mag_b = {-0.160726372652952, 0.093705049503890, 0.077222493617249};

		imu_filter.setConfiguration(imu_filter_conf);

		// mahony filter parameters
		mahony_conf.Kacc = 4.0f;
		mahony_conf.Kmag = 0.1f;
		mahony_conf.Kp   = 0.5f;
		mahony_conf.Ki   = 0.1f;

		mahony_filter.setConfiguration(mahony_conf);

		module.add(imu_filter);

		// Control node parameters
		control_node_conf.encoderTopicLeft = "encoder_left";
		control_node_conf.encoderTopicRight = "encoder_right";
		control_node_conf.motorTopicLeft = "torque_left";
		control_node_conf.motorTopicRight = "torque_right";
		control_node_conf.imuTopic = imu_filter_conf.topic;
		control_node_conf.frequency = imu_filter_conf.frequency;

		control_node_conf.L = 0.40;
		control_node_conf.R = 0.2;

		control_node_conf.K_theta = -130.1918;
		control_node_conf.K_omega = -51.0960;
		control_node_conf.K_omegaR = -3.0436;


		control_node_conf.K_linear = 3.55;
		control_node_conf.Ti_linear = 1.3498;
		control_node_conf.Td_linear = 0.1411;

		control_node_conf.K_angular = 10.0;

		control_node.setConfiguration(control_node_conf);

		module.add(control_node);

		// Setup and run
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
	}
}
