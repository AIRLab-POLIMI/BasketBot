#pragma once

#include <core/mw/CoreNode.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/Publisher.hpp>

#include <core/balancing_robot_control/ControlNodeConfiguration.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>
#include <core/sensor_msgs/Delta_f32.hpp>
#include <core/sensor_msgs/Imu.hpp>

#include <core/pid_ie/pid_ie.hpp>

namespace core
{
namespace balancing_robot_control {
class ControlNode: public core::mw::CoreNode,
				   public core::mw::CoreConfigurable<core::balancing_robot_control::ControlNodeConfiguration>::CoreConfigurable
{
public:
	ControlNode(const char* name, core::os::Thread::Priority priority =
			core::os::Thread::PriorityEnum::NORMAL);

	virtual
	~ControlNode();

public:
	ControlNodeConfiguration configuration;

private:
	bool
	onConfigure();

	bool
	onPrepareMW();

	bool
	onLoop();

private:
	float computeMeanTorque(float theta, float omega, float omegaR);
	float computeDifferentialTorque(float dPsi);
	float computePitch(const float orientation[4]);

private:
	core::mw::Publisher<actuator_msgs::Setpoint_f32> _mLeftPub;
	core::mw::Publisher<actuator_msgs::Setpoint_f32> _mRightPub;

	core::mw::Subscriber<sensor_msgs::Delta_f32, 5> _mLeftSub;
	core::mw::Subscriber<sensor_msgs::Delta_f32, 5> _mRightSub;
	core::mw::Subscriber<sensor_msgs::Imu, 5> _imuSub;

private:
	core::pid_ie::PID_IE _linearVelocityPID;
	core::pid_ie::PID_IE _angularVelocityPID;

	core::os::Time _Ts;
	core::os::Time _stamp;

};

}
}

