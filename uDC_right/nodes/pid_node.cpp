#include "ch.h"
#include "hal.h"

#include "r2p/Middleware.hpp"

#include "pid_node.hpp"
#include "pid.hpp"

/*
 * PID node
 */

bool enc_callback(const r2p::EncoderMsg &msg) {

	pwm = speed_pid.update(msg.delta * 50); // delta_rad to rad/s

	chSysLock();

	if (pwm > 0) {
		pwm_lld_enable_channel(&PWM_DRIVER, 1, pwm);
		pwm_lld_enable_channel(&PWM_DRIVER, 0, 0);
	} else {
		pwm_lld_enable_channel(&PWM_DRIVER, 1, 0);
		pwm_lld_enable_channel(&PWM_DRIVER, 0, -pwm);
	}
	chSysUnlock();

	return true;
}

msg_t pid_node(void * arg) {
	pid_conf * conf = reinterpret_cast<pid_conf *>(arg);
	r2p::Node node("pid");
	r2p::Subscriber<r2p::Speed2Msg, 5> speed_sub;
	r2p::Subscriber<r2p::EncoderMsg, 5> enc_sub(enc_callback);
	r2p::Subscriber<r2p::PIDCfgMsg, 5> cfg_sub;
	r2p::Speed2Msg * msgp;
	r2p::PIDCfgMsg *cfgp;
	r2p::Time last_setpoint(0);

	(void) arg;
	chRegSetThreadName("pid");

	speed_pid.config(conf->k, conf->ti, conf->td, conf->ts, -4095.0, 4095.0);

	/* Enable the h-bridge. */
	palSetPad(GPIOB, GPIOB_MOTOR_ENABLE); palClearPad(GPIOA, GPIOA_MOTOR_D1);
	chThdSleepMilliseconds(500);
	pwmStart(&PWM_DRIVER, &pwmcfg);

	node.subscribe(speed_sub, "speed2");
	node.subscribe(enc_sub, "encoder" MOTOR_STRING);
	node.subscribe(cfg_sub, "pidcfg");

	for (;;) {
		// Check for PID configuration parameters
		if (cfg_sub.fetch(cfgp)) {
			speed_pid.config(cfgp->k, cfgp->ti, cfgp->td, 0.02, -4095, 4095);
			cfg_sub.release(*cfgp);
		}

		if (node.spin(r2p::Time::ms(1000))) {
			// Check for speed setpoint
			if (speed_sub.fetch(msgp)) {
				speed_pid.set(msgp->value[MOTOR_ID]);
				last_setpoint = r2p::Time::now();
				speed_sub.release(*msgp);
			} else if (r2p::Time::now() - last_setpoint > r2p::Time::ms(1000)) {
				// Set speed to 0 if no speed messages for 1000 ms
				speed_pid.set(0);
			}
		} else {
			// Stop motor if no encoder messages for 1000 ms
			pwm_lld_disable_channel(&PWM_DRIVER, 0);
			pwm_lld_disable_channel(&PWM_DRIVER, 1);
		}
	}

	return CH_SUCCESS;
}
