#define USE_USB_SERIAL 1

#include "ch.h"
#include "hal.h"
#include "usbcfg.h"

#include <r2p/Middleware.hpp>
#include <r2p/msg/motor.hpp>
#include <r2p/msg/imu.hpp>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <r2p_msgs/ImuRaw.h>
#include <r2p_msgs/PidParameters.h>
#include <r2p_msgs/Vector3_32.h>


/*
 * R2P subscriber node.
 */
struct imu_data_t {
	float roll;
	float pitch;
	float yaw;
} imu_data;

struct imu_raw_data_t {
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
} imu_raw_data;

struct odometry_data_t {
	float x;
	float y;
	float w;
} odometry_data;

bool imu_cb(const r2p::IMUMsg &msg) {

	imu_data.roll = msg.roll;
	imu_data.pitch = msg.pitch;
	imu_data.yaw= msg.yaw;

	return true;
}

bool imu_raw_cb(const r2p::IMURaw9 &msg) {

	imu_raw_data.acc_x = msg.acc_x;
	imu_raw_data.acc_y = msg.acc_y;
	imu_raw_data.acc_z = msg.acc_z;
	imu_raw_data.gyro_x = msg.gyro_x;
	imu_raw_data.gyro_y = msg.gyro_y;
	imu_raw_data.gyro_z = msg.gyro_z;
	imu_raw_data.mag_x = msg.mag_x;
	imu_raw_data.mag_y = msg.mag_y;
	imu_raw_data.mag_z = msg.mag_z;

	return true;
}

bool odometry_cb(const r2p::Velocity3Msg &msg) {

	odometry_data.x = msg.x;
	odometry_data.y = msg.y;
	odometry_data.w= msg.w;

	return true;
}

msg_t r2p_sub_node(void * arg) {
	r2p::Node node("r2p_sub");
	r2p::Subscriber<r2p::IMUMsg, 5> imu_sub(imu_cb);
	r2p::Subscriber<r2p::IMURaw9, 5> imu_raw_sub(imu_raw_cb);
	r2p::Subscriber<r2p::Velocity3Msg, 5> odometry_sub(odometry_cb);

	(void) arg;
	chRegSetThreadName("r2p_sub");

	node.subscribe(imu_sub, "imu");
	node.subscribe(imu_raw_sub, "imu_raw");
	node.subscribe(odometry_sub, "odometry");

	for (;;) {
		node.spin(r2p::Time::ms(1000));
	}

	return CH_SUCCESS;
}

/*
 * ROS rosserial publisher thread.
 */

ros::NodeHandle nh;

float yaw = 0;
geometry_msgs::Vector3 odom_msg;

msg_t rosserial_pub_thread(void * arg) {
	r2p_msgs::Vector3_32 odometry_msg;
	r2p_msgs::Vector3_32 imu_msg;
	r2p_msgs::ImuRaw imu_raw_msg;
	ros::Publisher odometry_pub("odom", &odometry_msg);
	ros::Publisher imu_pub("imu", &imu_msg);
	ros::Publisher imu_raw_pub("imu_raw", &imu_raw_msg);
	systime_t last_sample;

	odom_msg.x = 0.0;
	odom_msg.y = 0.0;
	odom_msg.z = 0.0;

	(void) arg;
	chRegSetThreadName("rosserial_pub");

	nh.initNode();
	nh.advertise(odometry_pub);
	nh.advertise(imu_pub);
	nh.advertise(imu_raw_pub);

	for (;;) {
		last_sample = chTimeNow();

		odometry_msg.x = odometry_data.x;
		odometry_msg.y = odometry_data.y;
		odometry_msg.z = odometry_data.w;
		odometry_pub.publish(&odometry_msg);

		imu_msg.x = imu_data.roll;
		imu_msg.y = imu_data.pitch;
		imu_msg.z = imu_data.yaw;
		imu_pub.publish(&imu_msg);

		imu_raw_msg.linear_acceleration.x = imu_raw_data.acc_x;
		imu_raw_msg.linear_acceleration.y = imu_raw_data.acc_y;
		imu_raw_msg.linear_acceleration.z = imu_raw_data.acc_z;
		imu_raw_msg.angular_velocity.x = imu_raw_data.gyro_x;
		imu_raw_msg.angular_velocity.y = imu_raw_data.gyro_y;
		imu_raw_msg.angular_velocity.z = imu_raw_data.gyro_z;
		imu_raw_msg.magnetic_field.x = imu_raw_data.mag_x;
		imu_raw_msg.magnetic_field.y = imu_raw_data.mag_y;
		imu_raw_msg.magnetic_field.z = imu_raw_data.mag_z;
		imu_raw_pub.publish(&imu_raw_msg);

		nh.spinOnce();

		// Sleep for 50 milliseconds from last execution
		chThdSleepUntil(last_sample + MS2ST(50));
	}

	return CH_SUCCESS;
}


/*
 * ROS rosserial subscriber thread.
 */
r2p::Publisher<r2p::Velocity3Msg> vel_pub;
r2p::Publisher<r2p::PIDCfgMsg> balcfg_pub;
r2p::Publisher<r2p::PIDCfgMsg> velcfg_pub;

void cmd_vel_cb( const geometry_msgs::Twist& cmd_vel_msg){
	r2p::Velocity3Msg * msgp;

	if (vel_pub.alloc(msgp)) {
		msgp->x = cmd_vel_msg.linear.x;
		msgp->y = cmd_vel_msg.linear.y;
		msgp->w = cmd_vel_msg.angular.z;
		vel_pub.publish(*msgp);
	}
}

void balcfg_cb( const r2p_msgs::PidParameters& PID_config_msg){
	r2p::PIDCfgMsg * msgp;

	if (balcfg_pub.alloc(msgp)) {
		msgp->k = PID_config_msg.k;
		msgp->ti = PID_config_msg.ti;
		msgp->td = PID_config_msg.td;
		balcfg_pub.publish(*msgp);
	}
}

void velcfg_cb( const r2p_msgs::PidParameters& PID_config_msg){
	r2p::PIDCfgMsg * msgp;

	if (velcfg_pub.alloc(msgp)) {
		msgp->k = PID_config_msg.k;
		msgp->ti = PID_config_msg.ti;
		msgp->td = PID_config_msg.td;
		velcfg_pub.publish(*msgp);
	}
}

msg_t rosserial_sub_thread(void * arg) {
	r2p::Node r2p_node("r2p_pub", false);

	ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_cb );
	ros::Subscriber<r2p_msgs::PidParameters> balcfg_sub("balcfg", balcfg_cb );
	ros::Subscriber<r2p_msgs::PidParameters> velcfg_sub("velcfg", velcfg_cb );
	(void) arg;
	chRegSetThreadName("cmd_vel_sub");

	r2p_node.advertise(balcfg_pub, "balcfg", r2p::Time::INFINITE);
	r2p_node.advertise(velcfg_pub, "velcfg", r2p::Time::INFINITE);
	r2p_node.advertise(vel_pub, "velocity", r2p::Time::INFINITE);

	nh.initNode();
	nh.subscribe(cmd_vel_sub);
	nh.subscribe(balcfg_sub);
	nh.subscribe(velcfg_sub);

	for (;;) {
		nh.spinOnce();
		chThdSleepMilliseconds(5);
	}

	return CH_SUCCESS;
}
