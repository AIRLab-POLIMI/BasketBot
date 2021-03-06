#include <core/balancing_robot_control/ControlNode.hpp>
#include <math.h>

#include <core/quaternions/Utils.hpp>

namespace core
{
namespace balancing_robot_control {

ControlNode::ControlNode(const char* name, core::os::Thread::Priority priority) :
		CoreNode::CoreNode(name, priority),
		CoreConfigurable<core::balancing_robot_control::ControlNodeConfiguration>::CoreConfigurable(name)
{
	_workingAreaSize = 768;

	_Ts = 0;
}

ControlNode::~ControlNode() {

}

bool ControlNode::onConfigure() {
	//TODO use correct saturation limits
	_Ts = core::os::Time::hz(configuration().frequency);
	_stamp = core::os::Time::now();

	_linearVelocityPID.config(configuration().K_linear, configuration().Ti_linear,
			configuration().Td_linear, _Ts.to_s(), 100.0, -20.0, 20.0);

	_angularVelocityPID.config(configuration().K_angular, 0.0,
			0.0, 1.0/configuration().frequency, 100.0, -10.0, 10.0);


	_linearVelocityPID.set(0);
	_angularVelocityPID.set(0);
	return true;
}

bool ControlNode::onPrepareMW() {

	//publish motors setpoints
	advertise(_mLeftPub, configuration().motorTopicLeft);
	advertise(_mRightPub, configuration().motorTopicRight);

	//subscribe imu measurement
	subscribe(_imuSub, configuration().imuTopic);

	//subscribe motors setpoints
	subscribe(_mLeftSub, configuration().encoderTopicLeft);
	subscribe(_mRightSub, configuration().encoderTopicRight);

	return true;
}

bool ControlNode::onLoop() {

	core::os::Thread::sleep_until(_stamp+_Ts);

	sensor_msgs::Delta_f32* deltaLeft;
	sensor_msgs::Delta_f32* deltaRight;
	sensor_msgs::Imu* imu;

	// Wait for sensors readings
	while (!_mLeftSub.fetch(deltaLeft)) {
		//core::os::Thread::sleep(core::os::Time::ms(1));
	}

	while (!_mRightSub.fetch(deltaRight)) {
		//core::os::Thread::sleep(core::os::Time::ms(1));
	}

	while (!_imuSub.fetch(imu)) {
		//core::os::Thread::sleep(core::os::Time::ms(1));
	}

	//Compute wheels speeds //TODO change qei driver
	float speedLeft = deltaLeft->value * configuration().frequency;
	float speedRight = deltaRight->value * configuration().frequency;

	float omegaR = 0.5 * (speedLeft + speedRight);
	float dPsi = (speedRight - speedLeft) * configuration().R / configuration().L;

	//Compute orientation
	volatile float omega = imu->angular_velocity[1];
	float q[4];
	q[0]=imu->orientation[0];
	q[1]=imu->orientation[1];
	q[2]=imu->orientation[2];
	q[3]=imu->orientation[3];
	float theta = quaternions::Utils::getPitch(q);

	//Release messages
	_mLeftSub.release(*deltaLeft);
	_mRightSub.release(*deltaRight);
	_imuSub.release(*imu);

	//Linear velocity controller
	float tauM = computeMeanTorque(theta, omega, omegaR);

	//Angular velocity controller
	float tauD = computeDifferentialTorque(dPsi);

	//Mixing logic
	float tauL = 0.5 * (tauM - tauD);
	float tauR = 0.5 * (tauM + tauD);

	//publish setpoints
	actuator_msgs::Setpoint_f32* torqueLeft;
	actuator_msgs::Setpoint_f32* torqueRight;

	if (_mLeftPub.alloc(torqueLeft)) {
		torqueLeft->value = tauL;
		_mLeftPub.publish(torqueLeft);
	}

	if (_mRightPub.alloc(torqueRight)) {
		torqueRight->value = tauR;
		_mRightPub.publish(torqueRight);
	}

	_stamp = core::os::Time::now();

	return true;
}

float ControlNode::computeMeanTorque(float theta, float omega, float omegaR) {

	//Compute the stabilizing contribute
	const float& K_theta = configuration().K_theta;
	const float& K_omega = configuration().K_omega;
	const float& K_omegaR = configuration().K_omegaR;

	float stabilizingTorque = theta * K_theta + omega * K_omega
			- omegaR * K_omegaR;

	//Compute the speed contribute
	float linearVelocity = omegaR * configuration().R;
	float speedTorque = _linearVelocityPID.update(linearVelocity);

	//compute the mean torque
	return /*speedTorque*/ - stabilizingTorque;
}

float ControlNode::computeDifferentialTorque(float dPsi) {
	return _angularVelocityPID.update(dPsi);
}

}
}
