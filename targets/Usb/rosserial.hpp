#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/mw/CoreSensor.hpp>
#include <core/os/Callback.hpp>

#include <ModuleConfiguration.hpp>

#include <core/sensor_msgs/Imu.hpp>
#include <sensor_msgs/Imu.h>

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

private:
      bool
      onPrepareMW();

      bool
	  onStart();

      bool
      onLoop();

private:
    //Nova Core Subscribers
	core::mw::Subscriber<core::sensor_msgs::Imu, 5> _subscriber;

	//Nova core msg
	core::sensor_msgs::Imu core_msg;

	//ROS messages
	geometry_msgs::Vector3 ros_msg;

	//ROS publuishers
	ros::Publisher imu_pub;

	core::os::Time _stamp;

};

}
