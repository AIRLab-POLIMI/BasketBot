#include "rosserial.hpp"

#include <core/quaternions/Utils.hpp>

ros::NodeHandle nh;

const char* topicName = "imu";

const char* setpointName = "torque_left";

namespace rosserial {

std::function<void(const std_msgs::Float32&)> RosSerialPublisher::rosCallback;

RosSerialPublisher::RosSerialPublisher(const char* name,
		core::os::Thread::Priority priority) :
		CoreNode::CoreNode(name, priority),
		imu_pub("imu", &ros_imu_msg),
		setpoint_sub("cmd_vel", RosSerialPublisher::setpointCallback)
{
	_workingAreaSize = 512;
}

bool RosSerialPublisher::onPrepareMW() {
	rosCallback	= std::bind(&RosSerialPublisher::setpointCallbackPrivate, this, std::placeholders::_1);

	subscribe(_subscriber, topicName);
	_subscriber.set_callback(imuCallback);

	advertise(_publisher, setpointName);

	return true;
}

bool RosSerialPublisher::imuCallback(
	   const core::sensor_msgs::Imu& msg,
	   core::mw::Node*               node)
{
	RosSerialPublisher* tmp = static_cast<RosSerialPublisher*>(node);

	tmp->core_imu_msg = msg;

   return true;
}

bool RosSerialPublisher::onLoop() {

	if(this->spin(core::os::Time::ms(1)))
	{

		float q[4];
		q[0]=core_imu_msg.orientation[0];
		q[1]=core_imu_msg.orientation[1];
		q[2]=core_imu_msg.orientation[2];
		q[3]=core_imu_msg.orientation[3];

		ros_imu_msg.x = quaternions::Utils::getRoll(q) * 180 / 3.14;
		ros_imu_msg.y = quaternions::Utils::getPitch(q) * 180 / 3.14;
		ros_imu_msg.z = quaternions::Utils::getYaw(q) * 180 / 3.14;

		imu_pub.publish(&ros_imu_msg);
	}

	nh.spinOnce();


	return true;
}

bool RosSerialPublisher::onStart() {
	nh.initNode();
	nh.advertise(imu_pub);
	nh.subscribe(setpoint_sub);

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
