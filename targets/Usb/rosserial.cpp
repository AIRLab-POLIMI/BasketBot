#define USE_USB_SERIAL 1

#include "ch.h"
#include "hal.h"
#include "usbcfg.h"

#include <Configuration.hpp>
#include <Module.hpp>
#include <Core/MW/Publisher.hpp>
#include <Core/MW/Subscriber.hpp>

#include <sensor_msgs/Imu_f32.hpp>
#include <sensor_msgs/Imu.h>

#include <actuator_msgs/Setpoint_f32.hpp>

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

/*
 * ROS rosserial publisher thread.
 */

ros::NodeHandle nh;

sensor_msgs::Imu_f32 imu_msg;

bool imu_cb(const sensor_msgs::Imu_f32 &msg, Core::MW::Node* node) {
	imu_msg = msg;

	return true;
}


void rosserial_pub_thread(void * arg) {
	Core::MW::Node node("rosserial_pub");
	Core::MW::Subscriber<sensor_msgs::Imu_f32, 5> imu_sub(imu_cb);

	sensor_msgs::Imu ros_imu_msg;
	ros::Publisher imu_pub("imu", &ros_imu_msg);

	std_msgs::String ros_string_msg;
	ros::Publisher test_pub("test", &ros_string_msg);

	(void) arg;
	chRegSetThreadName("rosserial_pub");

	node.subscribe(imu_sub, "imu");

	nh.initNode();
	//nh.advertise(imu_pub);
	nh.advertise(test_pub);

	nh.spinOnce();

	for (;;) {
		if (node.spin(Core::MW::Time::ms(500))) {
			ros_imu_msg.orientation.x = imu_msg.orientation[0];
			ros_imu_msg.orientation.y = imu_msg.orientation[1];
			ros_imu_msg.orientation.z = imu_msg.orientation[2];
			ros_imu_msg.orientation.w = imu_msg.orientation[3];

			//TODO linear

			//TODO angular

			ros_string_msg.data = std::string(200, '#').c_str();
			test_pub.publish(&ros_string_msg);

			//imu_pub.publish(&ros_imu_msg);

			nh.spinOnce();
		}

		Core::MW::Thread::sleep(Core::MW::Time::ms(10));
	}
}


/*
 * ROS rosserial subscriber thread.
 */
Core::MW::Publisher<actuator_msgs::Setpoint_f32> setpoint_pub;

void setpoint_cb( const std_msgs::Float32& cmd_vel_msg){
	actuator_msgs::Setpoint_f32 * msgp;

	if (setpoint_pub.alloc(msgp)) {
		msgp->value = cmd_vel_msg.data;
		setpoint_pub.publish(*msgp);
	}
}

void rosserial_sub_thread(void * arg) {
	Core::MW::Node node("rosserial_sub", false);
	ros::Subscriber<std_msgs::Float32> setpoint_sub("cmd_vel", &setpoint_cb );

	(void) arg;
	chRegSetThreadName("rosserial_sub");

	node.advertise(setpoint_pub, "cmd_vel");

	nh.initNode();
	nh.subscribe(setpoint_sub);

	for (;;) {
		nh.spinOnce();
		chThdSleepMilliseconds(5);
	}
}
