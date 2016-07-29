#pragma once

#include <Core/MW/CoreNode.hpp>
#include <Core/MW/Subscriber.hpp>
#include <Core/MW/Publisher.hpp>

#include <balancing_robot_control/ControlNodeConfiguration.hpp>
#include <actuator_msgs/Setpoint_f32.hpp>
#include <sensor_msgs/Imu_f32.hpp>

#include <pid_ie/pid_ie.hpp>

namespace balancing_robot_control {
class ControlNode: public Core::MW::CoreNode {
public:
	ControlNode(const char* name, Core::MW::Thread::Priority priority =
			Core::MW::Thread::PriorityEnum::NORMAL);

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
	float computePitch(float orientation[4]);

private:
	Core::MW::Publisher<actuator_msgs::Setpoint_f32> _mLeftPub;
	Core::MW::Publisher<actuator_msgs::Setpoint_f32> _mRightPub;

	Core::MW::Subscriber<sensor_msgs::Delta_f32, 5> _mLeftSub;
	Core::MW::Subscriber<sensor_msgs::Delta_f32, 5> _mRightSub;
	Core::MW::Subscriber<sensor_msgs::Imu_f32, 5> _imuSub;

private:
	pid_ie::PID_IE linearVelocityPID;
	pid_ie::PID_IE angularVelocityPID;
	float K_theta, K_omega, K_omegaR;

};

}

