/*#define USE_USB_SERIAL 1

#include "ch.h"
#include "hal.h"
#include "usbcfg.h"*/

#include "rosserial.hpp"

ros::NodeHandle nh;

core::sensor_msgs::Imu imu_msg;

const char* topicName = "imu";

namespace rosserial {

RosSerialPublisher::RosSerialPublisher(const char* name,
		core::os::Thread::Priority priority) :
		CoreNode::CoreNode(name, priority), imu_pub("imu", &ros_imu_msg) {
	_workingAreaSize = 512;

}

bool RosSerialPublisher::onPrepareMW() {
	subscribe(_subscriber, topicName);

	return true;
}

bool RosSerialPublisher::onLoop() {
	if (spin(core::os::Time::ms(500))) {
		ros_imu_msg.orientation.x = imu_msg.orientation[0];
		ros_imu_msg.orientation.y = imu_msg.orientation[1];
		ros_imu_msg.orientation.z = imu_msg.orientation[2];
		ros_imu_msg.orientation.w = imu_msg.orientation[3];

		//TODO linear

		//TODO angular

		imu_pub.publish(&ros_imu_msg);

	}

	nh.spinOnce();
	core::os::Thread::sleep(core::os::Time::ms(10));

	return true;
}

bool RosSerialPublisher::onStart() {
	nh.initNode();
	nh.advertise(imu_pub);

	nh.spinOnce();

	return true;
}

}

/*bool imu_cb(const core::sensor_msgs::Imu &msg, core::mw::Node* node) {
 imu_msg = msg;

 return true;
 }


 void rosserial_pub_thread(void * arg) {
 core::mw::Node node("rosserial_pub");
 core::mw::Subscriber<core::sensor_msgs::Imu, 5> imu_sub(imu_cb);

 sensor_msgs::Imu ros_imu_msg;
 ros::Publisher imu_pub("imu", &ros_imu_msg);

 std_msgs::String ros_string_msg;
 ros::Publisher test_pub("test", &ros_string_msg);

 (void) arg;
 chRegSetThreadName("rosserial_pub");

 node.subscribe(imu_sub, "imu");

 nh.initNode();
 //nh.advertise(imu_pub);
 nh.advertise(test_pub);

 nh.spinOnce();

 for (;;) {
 //if (node.spin(core::os::Time::ms(500))) {
 core::os::Thread::sleep(core::os::Time::ms(10));
 Module::led.toggle();
 ros_imu_msg.orientation.x = imu_msg.orientation[0];
 ros_imu_msg.orientation.y = imu_msg.orientation[1];
 ros_imu_msg.orientation.z = imu_msg.orientation[2];
 ros_imu_msg.orientation.w = imu_msg.orientation[3];

 //TODO linear

 //TODO angular

 ros_string_msg.data = std::string(10, '#').c_str();
 test_pub.publish(&ros_string_msg);

 //imu_pub.publish(&ros_imu_msg);

 nh.spinOnce();
 //}

 //core::os::Thread::sleep(core::os::Time::ms(10));
 }
 }

 void rosserial_test_thread(void * arg) {
 core::mw::Node node("rosserial_pub");
 sensor_msgs::Imu ros_imu_msg;
 ros::Publisher imu_pub("imu", &ros_imu_msg);

 (void) arg;
 chRegSetThreadName("rosserial_pub");

 nh.initNode();
 nh.advertise(imu_pub);

 nh.spinOnce();

 for (;;) {namespace core {
 namespace rosserial {
 core::os::Thread::sleep(core::os::Time::ms(10));

 ros_imu_msg.orientation.x = 1;
 ros_imu_msg.orientation.y = 2;
 ros_imu_msg.orientation.z = 3;
 ros_imu_msg.orientation.w = 4;
 imu_pub.publish(&ros_imu_msg);

 nh.spinOnce();
 }
 }

 core::mw::Publisher<core::actuator_msgs::Setpoint_f32> setpoint_pub;

 void setpoint_cb( const std_msgs::Float32& cmd_vel_msg){
 core::actuator_msgs::Setpoint_f32* msgp;

 if (setpoint_pub.alloc(msgp)) {
 msgp->value = cmd_vel_msg.data;
 setpoint_pub.publish(*msgp);
 }
 }

 void rosserial_sub_thread(void * arg) {
 core::mw::Node node("rosserial_sub", false);
 ros::Subscriber<std_msgs::Float32> setpoint_sub("cmd_vel", &setpoint_cb );

 (void) arg;
 chRegSetThreadName("rosserial_sub");

 node.advertise(setpoint_pub, "cmd_vel");

 nh.initNode();
 nh.subscribe(setpoint_sub);

 for (;;) {
 nh.spinOnce();
 chThdSleepMilliseconds(5);
 }
 }*/
