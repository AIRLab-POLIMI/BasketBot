#include "ch.h"
#include "hal.h"

#include <r2p/Middleware.hpp>
#include <r2p/node/led.hpp>
#include <r2p/msg/motor.hpp>
#include <r2p/msg/imu.hpp>

#include "nodes/madgwick_node.hpp"
#include "pid.hpp"

#include "ExtraMsgs.h"

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "IMU"
#endif

static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);


/*===========================================================================*/
/* Kinematics.                                                               */
/*===========================================================================*/
/*
 *               y
 *               ^
 *               |
 *               |
 *   2           @---->x     1
 *  ||                       ||  |
 *  ||_______________________||  | R
 *  ||                       ||
 *  ||                       ||
 *               L
 *   <----------------------->
 *
 */

// Robot parameters
#define _L        0.220f     // Wheel distance [m]
#define _R        (0.155f / 2) // Wheel radius [m]


/*
 * Velocity control node
 */
PID vel_pid;
float vel_setpoint = 0;
float w_setpoint = 0;
float angle_setpoint = 0;
float right_speed = 0;
float left_speed = 0;

bool enc0_callback(const r2p::EncoderMsg &msg) {
	right_speed = -msg.delta * _R * 20; // ->m/s @20hz

	return true;
}

bool enc1_callback(const r2p::EncoderMsg &msg) {
	left_speed = msg.delta * _R * 20; // ->m/s @20hz

	return true;
}

msg_t velocity_node(void *arg) {
	r2p::Node node("velocity");
	r2p::Subscriber<r2p::Velocity3Msg, 5> vel_sub;
	r2p::Velocity3Msg *velp;
	r2p::Subscriber<r2p::EncoderMsg, 10> enc0_sub(enc0_callback);
	r2p::Subscriber<r2p::EncoderMsg, 10> enc1_sub(enc1_callback);
	r2p::Publisher<r2p::Velocity3Msg> odometry_pub;
	r2p::Subscriber<r2p::PIDCfgMsg, 5> cfg_sub;
	r2p::PIDCfgMsg *cfgp;
	float v;
	float w;

	(void) arg;
	chRegSetThreadName("velocity");

	node.subscribe(enc0_sub, "encoder0");
	node.subscribe(enc1_sub, "encoder1");
	node.subscribe(vel_sub, "velocity");
	node.subscribe(cfg_sub, "velcfg");

	node.advertise(odometry_pub, "odometry");

	vel_pid.config(2.0, 0.0, -0.1, 0.05, -10.0, 10.0);
	vel_pid.set(0);

	for (;;) {
		if (!node.spin(r2p::Time::ms(500))) {
			angle_setpoint = 0;
			continue;
		}

		v = (left_speed + right_speed) / 2;
		w = (right_speed - left_speed) / _L;
		angle_setpoint = vel_pid.update(v); // 20hz

		r2p::Velocity3Msg *msgp;
		if (odometry_pub.alloc(msgp)) {
			msgp->x = v;
			msgp->y = 0;
			msgp->w = w;
			odometry_pub.publish(*msgp);
		}

		while (vel_sub.fetch(velp)) {
			vel_setpoint = velp->x;
			w_setpoint = velp->w;
			vel_sub.release(*velp);
		}
		vel_pid.set(vel_setpoint);

		while (cfg_sub.fetch(cfgp)) {
			vel_pid.config(cfgp->k, cfgp->ti, cfgp->td, 0.05, -10, 10);
			cfg_sub.release(*cfgp);
		}

	}
	return CH_SUCCESS;
}

/*
 * Balance control node
 */
PID balance_pid;

msg_t balance_node(void *arg) {
	r2p::Node node("balance");
	r2p::Subscriber<r2p::IMUMsg, 5> imu_sub;
	r2p::Subscriber<r2p::PIDCfgMsg, 5> cfg_sub;
	r2p::IMUMsg *msgp;
	r2p::PIDCfgMsg *cfgp;

	r2p::Publisher<r2p::PWM2Msg> pwm_pub;
	r2p::PWM2Msg *pwmp;

	int32_t pwm = 0;

	(void) arg;
	chRegSetThreadName("balance");

	node.advertise(pwm_pub, "pwm");
	node.subscribe(imu_sub, "imu");
	node.subscribe(cfg_sub, "balcfg");


	balance_pid.config(250, 0.2, 0.02, 0.02, -4000, 4000);
	balance_pid.set(angle_setpoint);

	for (;;) {
		balance_pid.set(angle_setpoint);

		while (!imu_sub.fetch(msgp)) {
			r2p::Thread::sleep(r2p::Time::ms(1));
		}

		while (cfg_sub.fetch(cfgp)) {
			balance_pid.config(cfgp->k, cfgp->ti, cfgp->td, 0.02, -4000, 4000);
			cfg_sub.release(*cfgp);
		}

		pwm = balance_pid.update(msgp->pitch + 0.7); // tilty offset
		imu_sub.release(*msgp);

		if (pwm_pub.alloc(pwmp)) {
			pwmp->value[0] = pwm - (w_setpoint * 200);
			pwmp->value[1] = -(pwm + (w_setpoint * 200));
			pwm_pub.publish(*pwmp);
		}
	}
	return CH_SUCCESS;
}

/*msg_t test_speed_node(void *arg) {
	(void) arg;

	r2p::Node vel_node("uvelpub", false);
	r2p::Publisher<r2p::Current2Msg> vel_pub;

	vel_node.advertise(vel_pub, "current2", r2p::Time::INFINITE);
    vel_node.set_enabled(true);

	r2p::Current2Msg* msgp;

	systime_t time;
	while(true)
	{
		time = chTimeNow();

		if (vel_pub.alloc(msgp)) {
			msgp->value[0] = 0.0;
			msgp->value[1] = 0.0;
		}

		time += MS2ST(50);
		chThdSleepUntil(time);
	}

	return CH_SUCCESS;
}*/

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
	rtcantra.initialize(rtcan_config);
	r2p::Middleware::instance.start();

	r2p::ledpub_conf ledpub_conf = {"led", 1};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledpub_node, &ledpub_conf);

	r2p::ledsub_conf ledsub_conf = { "led" };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 3, madgwick_node, NULL);
	r2p::Thread::sleep(r2p::Time::ms(2000));

	//r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 2, balance_node, NULL);
	//r2p::Thread::sleep(r2p::Time::ms(500));
	//r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, velocity_node, NULL);
	//r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, test_speed_node, NULL);

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
}

