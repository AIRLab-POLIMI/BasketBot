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


#define _R                 0.299f
#define _L                 8.2e-5f

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define INDEX 1

//#define CALIBRATION
#define CURRENT


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

	r2p::ledsub_conf ledsub_conf = { "led" };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(256), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);

	encoder_node_conf encoder_conf = {"encoder_node", "encoder" STR(INDEX), R2T};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 2, encoder_node, &encoder_conf);

#ifdef CURRENT
	r2p::broadcaster_node_conf broadcaster_conf = { "broadcaster_node", "current_measure" STR(INDEX), "bits_packed", 50 };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO, r2p::broadcaster_node, &broadcaster_conf);
	r2p::current_publisher_node_conf publisher_conf = { "current_publisher", "current_measure" STR(INDEX) };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO, r2p::current_publisher_node, &publisher_conf);
#endif

#ifdef CALIBRATION
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 2, r2p::motor_calibration_node, NULL);
#else
	r2p::current_pid_node_conf pid_conf = { "current_pid" STR(INDEX), INDEX, _R, _L, 6000.0f, 24.0f};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO + 3, r2p::current_pid2_node, &pid_conf);
#endif

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
}
