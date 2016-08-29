#include "rosserial.hpp"

#include <core/quaternions/Utils.hpp>

ros::NodeHandle nh;

const char* imuTopic = "imu";
const char* currentLeftTopic = "current_left";
const char* currentRightTopic = "current_right";
const char* leftName = "encoder_left";
const char* rightName = "encoder_right";

const char* setpointName = "cmd_vel";

const float encoderFrequency = 100;

namespace rosserial {

std::function<void(const std_msgs::Float32&)> RosSerialPublisher::rosCallback;

RosSerialPublisher::RosSerialPublisher(const char* name,
		core::os::Thread::Priority priority) :
		CoreNode::CoreNode(name, priority),
		imu_pub(imuTopic, &ros_imu_msg),
		current_left_pub(currentLeftTopic, &ros_current_left_msg),
		current_right_pub(currentRightTopic, &ros_current_right_msg),
		encoder_left_pub(leftName, &ros_left_msg),
		encoder_right_pub(rightName, &ros_right_msg),
		setpoint_sub(setpointName, RosSerialPublisher::setpointCallback)
{
	_workingAreaSize = 512;

	imuNew = false;
	currentLeft = false;
	currentRight = false;
	encoderLeft = false;
	encoderRight = false;
}

bool RosSerialPublisher::onPrepareMW() {
	rosCallback	= std::bind(&RosSerialPublisher::setpointCallbackPrivate, this, std::placeholders::_1);

	subscribe(_subscriberImu, imuTopic);
	_subscriberImu.set_callback(imuCallback);

	subscribe(_subscriberCurrentLeft, currentLeftTopic);
	_subscriberCurrentLeft.set_callback(currentLeftCallback);

	subscribe(_subscriberCurrentRight, currentRightTopic);
	_subscriberCurrentRight.set_callback(currentRightCallback);

	subscribe(_subscriberLeft, leftName);
	_subscriberLeft.set_callback(encoderLeftCallback);

	subscribe(_subscriberRight, rightName);
	_subscriberRight.set_callback(encoderRightCallback);


	advertise(_publisher, setpointName);

	return true;
}

bool RosSerialPublisher::imuCallback(
	   const core::sensor_msgs::Imu& msg,
	   core::mw::Node*               node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	float q[4];
	q[0]=msg.orientation[0];
	q[1]=msg.orientation[1];
	q[2]=msg.orientation[2];
	q[3]=msg.orientation[3];

	tmp->ros_imu_msg.x = quaternions::Utils::getRoll(q) * 180 / 3.14;
	tmp->ros_imu_msg.y = quaternions::Utils::getPitch(q) * 180 / 3.14;
	tmp->ros_imu_msg.z = quaternions::Utils::getYaw(q) * 180 / 3.14;

	tmp->imuNew = true;

   return true;
}

bool RosSerialPublisher::currentLeftCallback(
	   const core::actuator_msgs::Setpoint_f32& msg,
	   core::mw::Node* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	tmp->ros_current_left_msg.data = msg.value;

	tmp->currentLeft = true;

   return true;
}

bool RosSerialPublisher::currentRightCallback(
	   const core::actuator_msgs::Setpoint_f32& msg,
	   core::mw::Node* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	tmp->ros_current_right_msg.data = msg.value;

	tmp->currentRight = true;

   return true;
}

bool RosSerialPublisher::encoderLeftCallback(const core::sensor_msgs::Delta_f32& msg,
			   core::mw::Node* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);


	tmp->ros_left_msg.data = msg.value*encoderFrequency;
	tmp->encoderLeft = true;

	return true;
}

bool RosSerialPublisher::encoderRightCallback(const core::sensor_msgs::Delta_f32& msg,
				   core::mw::Node* node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);


	tmp->ros_right_msg.data = msg.value*encoderFrequency;
	tmp->encoderRight = true;

	return true;
}

bool RosSerialPublisher::onLoop() {

	if(this->spin(core::os::Time::ms(1)))
	{
		if(imuNew)
		{
			imu_pub.publish(&ros_imu_msg);
			imuNew = false;
		}

		if(currentLeft)
		{
			current_left_pub.publish(&ros_current_left_msg);
			currentLeft = false;
		}

		if(currentRight)
		{
			current_right_pub.publish(&ros_current_right_msg);
			currentRight = false;
		}

		if(encoderLeft)
		{
			encoder_left_pub.publish(&ros_left_msg);
			encoderLeft = false;
		}

		if(encoderRight)
		{
			encoder_right_pub.publish(&ros_right_msg);
			encoderRight = false;
		}
	}

	nh.spinOnce();


	return true;
}

bool RosSerialPublisher::onStart() {
	nh.initNode();
	nh.advertise(imu_pub);
	nh.advertise(current_left_pub);
	nh.advertise(current_right_pub);
	nh.subscribe(setpoint_sub);
	nh.advertise(encoder_left_pub);
	nh.advertise(encoder_right_pub);

	nh.spinOnce();
	core::os::Thread::sleep(core::os::Time::ms(100));

	return true;
}


void RosSerialPublisher::setpointCallback(const std_msgs::Float32& setpoint_msg)
{
	rosCallback(setpoint_msg);
}

void RosSerialPublisher::setpointCallbackPrivate(const std_msgs::Float32& setpoint_msg)
{
	 core::actuator_msgs::Setpoint_f32* msgp;

	 if (_publisher.alloc(msgp)) {
		 msgp->value = setpoint_msg.data;
		 _publisher.publish(*msgp);
	 }
}

}
