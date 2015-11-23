#include "ch.h"
#include "hal.h"
#include "qei.h"

#include "rtcan.h"

#include <r2p/Middleware.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "led/nodes/led.hpp"

#include "nodes/current_pid_node.hpp"
#include "nodes/encoder_node.hpp"
#include "nodes/broadcaster_node.hpp"
#include "nodes/calibration_node.hpp"

#define _TICKS 48.0f
#define _RATIO 19.0f
#define _PI 3.14159265359f

#define MOTOR

#ifdef MOTOR
	#define _R                 0.115f
	#define _L                 2.4e-5f
#else
	#define _R                 8.2f
	#define _L                 1.17e-3f
#endif

//#define CALIBRATION


#define R2T ((1 / (2 * _PI)) * (_TICKS * _RATIO))

static WORKING_AREA(wa_info, 2048);

r2p::Middleware r2p::Middleware::instance(MODULE_NAME, "BOOT_"MODULE_NAME);

// RTCAN transport
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	qeiInit();
	chSysInit();

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);

	rtcantra.initialize(rtcan_config);

	r2p::Middleware::instance.start();

	r2p::ledsub_conf ledsub_conf = { "leds" };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(256), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);

	encoder_node_conf encoder_conf = {"encoder_node", "encoder0", R2T};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 2, encoder_node, &encoder_conf);

#ifdef CALIBRATION
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 2, r2p::broadcaster_node, NULL);
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 2, r2p::motor_calibration_node, NULL);
#else
	r2p::broadcaster_node_conf calibration_conf = { "calibration_node", "current_measure1", "bits_packed", 20 };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO, r2p::broadcaster_node, &calibration_conf);
	r2p::current_pid_node_conf pid_conf = { "current_pid1",  "current_measure1", 0, _R, _L, 6000.0f, 24.0f};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 3, r2p::current_pid2_node, &pid_conf);
#endif

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
}
