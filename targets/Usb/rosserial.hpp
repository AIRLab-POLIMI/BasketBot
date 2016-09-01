#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/mw/CoreSensor.hpp>
#include <core/os/Callback.hpp>

#include <ModuleConfiguration.hpp>

//Core msgs
#include <core/sensor_msgs/Imu.hpp>
#include <core/sensor_msgs/Delta_f32.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

//ROS msgs
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#define USE_USB_SERIAL 1
#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <ros.h>

namespace rosserial {
class RosSerialPublisher: public core::mw::CoreNode {
public:
	RosSerialPublisher(const char* name,
	         core::os::Thread::Priority priority = core::os::Thread::PriorityEnum::NORMAL);

public:
	static bool
	imuCallback(
	   const core::sensor_msgs::Imu& msg,
	   core::mw::Node*               node);


	static void
	setpointCallback(const std_msgs::Float32& setpoint_msg);

	static bool
	currentLeftCallback(const core::actuator_msgs::Setpoint_f32& msg,
				   core::mw::Node* node);

	static bool
	currentRightCallback(const core::actuator_msgs::Setpoint_f32& msg,
					   core::mw::Node* node);

	static bool
	torqueLeftCallback(const core::actuator_msgs::Setpoint_f32& msg,
				   core::mw::Node* node);

	static bool
	torqueRightCallback(const core::actuator_msgs::Setpoint_f32& msg,
					   core::mw::Node* node);



	static bool
	encoderLeftCallback(const core::sensor_msgs::Delta_f32& msg,
				   core::mw::Node* node);

	static bool
	encoderRightCallback(const core::sensor_msgs::Delta_f32& msg,
					   core::mw::Node* node);

private:
	void setpointCallbackPrivate(const std_msgs::Float32& setpoint_msg);

private:
      bool
      onPrepareMW();

      bool
	  onStart();

      bool
      onLoop();

private:

      static std::function<void(const std_msgs::Float32&)> rosCallback;

private:
    //Nova Core
	core::mw::Subscriber<core::sensor_msgs::Imu, 5> _subscriberImu;
	core::mw::Subscriber<core::actuator_msgs::Setpoint_f32, 5> _subscriberCurrentLeft;
	core::mw::Subscriber<core::actuator_msgs::Setpoint_f32, 5> _subscriberCurrentRight;
	core::mw::Subscriber<core::actuator_msgs::Setpoint_f32, 5> _subscriberTorqueLeft;
	core::mw::Subscriber<core::actuator_msgs::Setpoint_f32, 5> _subscriberTorqueRight;
	core::mw::Subscriber<core::sensor_msgs::Delta_f32, 5> _subscriberLeft;
	core::mw::Subscriber<core::sensor_msgs::Delta_f32, 5> _subscriberRight;
	core::mw::Publisher<core::actuator_msgs::Setpoint_f32> _publisher;

	bool currentLeft;
	bool currentRight;
	bool torqueLeft;
	bool torqueRight;
	bool imuNew;
	bool encoderLeft;
	bool encoderRight;

	//ROS
	geometry_msgs::Vector3 ros_imu_msg;
	std_msgs::Float32 ros_current_left_msg;
	std_msgs::Float32 ros_current_right_msg;
	std_msgs::Float32 ros_torque_left_msg;
	std_msgs::Float32 ros_torque_right_msg;
	std_msgs::Float32 ros_left_msg;
	std_msgs::Float32 ros_right_msg;

	ros::Publisher imu_pub;
	ros::Publisher current_left_pub;
	ros::Publisher current_right_pub;
	ros::Publisher torque_left_pub;
	ros::Publisher torque_right_pub;
	ros::Publisher encoder_left_pub;
	ros::Publisher encoder_right_pub;
	ros::Subscriber<std_msgs::Float32> setpoint_sub;

};

}
