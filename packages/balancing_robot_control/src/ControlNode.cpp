#include <balancing_robot_control/ControlNode.hpp>
#include <cmath>

namespace balancing_robot_control {

ControlNode::ControlNode(const char* name, Core::MW::Thread::Priority priority) :
		CoreNode::CoreNode(name, priority) {
	_workingAreaSize = 512;
	K_theta = 0;
	K_omega = 0;
	K_omegaR = 0;
}

ControlNode::~ControlNode() {

}

bool ControlNode::onConfigure() {

	return true;
}

bool ControlNode::onPrepareMW() {

	//publish motors setpoints
	const char* motorTopic = configuration.motorTopic;
	std::string topic1(motorTopic);
	topic1 += "_left";

	std::string topic2(motorTopic);
	topic2 += "_right";

	this->advertise(_mLeftPub, topic1.c_str());
	this->advertise(_mRightPub, topic2.c_str());

	//subscribe imu measurement
	subscribe(_imuSub, configuration.imuTopic);

	return true;
}

bool ControlNode::onLoop() {

	sensor_msgs::Delta_f32* deltaLeft;
	sensor_msgs::Delta_f32* deltaRight;
	sensor_msgs::Imu_f32* imu;

	// Wait for sensors readings
	while (!_mLeftSub.fetch(deltaLeft)) {
		Core::MW::Thread::sleep(Core::MW::Time::ms(1));
	}

	while (!_mLeftSub.fetch(deltaRight)) {
		Core::MW::Thread::sleep(Core::MW::Time::ms(1));
	}

	while (!_imuSub.fetch(imu)) {
		Core::MW::Thread::sleep(Core::MW::Time::ms(1));
	}

	//Compute wheels speeds
	float frequency = 50; //TODO change QEI driver
	float speedLeft = deltaLeft->value * frequency;
	float speedRight = deltaRight->value * frequency;

	float omegaR = 0.5 * (speedLeft + speedRight);
	float dPsi = (speedRight - speedLeft) * configuration.R / configuration.L;

	//Compute orientation
	float omega = imu->angular_velocity[1];
	float theta = computePitch(imu->orientation);

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
		_mLeftPub.publish(torqueRight);
	}

	//Release messages
	_mLeftSub.release(*deltaLeft);
	_mRightSub.release(*deltaRight);
	_imuSub.release(*imu);

	return true;
}

float ControlNode::computeMeanTorque(float theta, float omega, float omegaR) {

	//Compute the stabilizing contribute
	float stabilizingTorque = theta * K_theta + omega * K_omega
			+ omegaR * K_omegaR;

	//Compute the speed contribute
	float linearVelocity = omegaR * configuration.R;
	float speedTorque = linearVelocityPID.update(linearVelocity);

	//compute the mean torque
	return speedTorque - stabilizingTorque;
}

float ControlNode::computeDifferentialTorque(float dPsi) {
	//TODO implement
	return 0;
}

float ControlNode::computePitch(float q[4]) {
	return std::asin(-2.0*(q[0]*q[2] - q[3]*q[1]));

}

}
