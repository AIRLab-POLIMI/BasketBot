#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/mw/CoreSensor.hpp>
#include <core/os/Callback.hpp>

#include <ModuleConfiguration.hpp>

//Core msgs
#include <core/sensor_msgs/Imu.hpp>
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
	currentCallback(const core::actuator_msgs::Setpoint_f32& msg,
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
	core::mw::Subscriber<core::actuator_msgs::Setpoint_f32, 5> _subscriberCurrent;
	core::mw::Publisher<core::actuator_msgs::Setpoint_f32> _publisher;

	bool currentNew;
	bool imuNew;

	//ROS
	geometry_msgs::Vector3 ros_imu_msg;
	std_msgs::Float32 ros_current_msg;

	ros::Publisher imu_pub;
	ros::Publisher current_pub;
	ros::Subscriber<std_msgs::Float32> setpoint_sub;

};

}
