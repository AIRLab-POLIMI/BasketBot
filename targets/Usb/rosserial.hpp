#pragma once

#include <ros.h>

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/mw/CoreSensor.hpp>
#include <core/os/Callback.hpp>

#include <ModuleConfiguration.hpp>

#include <core/sensor_msgs/Imu.hpp>
#include <sensor_msgs/Imu.h>

namespace rosserial {
class RosSerialPublisher: public core::mw::CoreNode {
public:
	RosSerialPublisher(const char* name,
	         core::os::Thread::Priority priority = core::os::Thread::PriorityEnum::NORMAL);

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

	//ROS messages
	sensor_msgs::Imu ros_imu_msg;

	//ROS publuishers
	ros::Publisher imu_pub;

};

}
