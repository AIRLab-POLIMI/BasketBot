#include <stdint.h> // atof()

#include "ch.h"
#include "hal.h"

#include "usbcfg.h"

#include <r2p/Middleware.hpp>
#include <r2p/node/led.hpp>

#include "nodes/rosserial.hpp"

static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(MODULE_NAME, "BOOT_"MODULE_NAME);

/*
 * Cycle USB connection on power up.
 */

void usb_lld_disconnect_bus(USBDriver *usbp) {

	(void)usbp;
	palClearPort(GPIOA, (1<<GPIOA_USB_DM) | (1<<GPIOA_USB_DP));
	palSetPadMode(GPIOA, GPIOA_USB_DM, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOA, GPIOA_USB_DP, PAL_MODE_OUTPUT_PUSHPULL);
}

void usb_lld_connect_bus(USBDriver *usbp) {

	(void)usbp;
	palClearPort(GPIOA, (1<<GPIOA_USB_DM) | (1<<GPIOA_USB_DP));
	palSetPadMode(GPIOA, GPIOA_USB_DM, PAL_MODE_ALTERNATE(14));
	palSetPadMode(GPIOA, GPIOA_USB_DP, PAL_MODE_ALTERNATE(14));
}

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	/*
	 * Initializes a serial-over-USB CDC driver.
	 */
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg);

	/*
	 * Activates the USB driver and then the USB bus pull-up on D+.
	 * Note, a delay is inserted in order to not have to disconnect the cable
	 * after a reset.
	 */
	usbDisconnectBus(serusbcfg.usbp);
	chThdSleepMilliseconds(500);
	usbStart(serusbcfg.usbp, &usbcfg);
	usbConnectBus(serusbcfg.usbp);

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
	rtcantra.initialize(rtcan_config);
	r2p::Middleware::instance.start();

	r2p::ledsub_conf ledsub_conf = {"led"};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(1024), NORMALPRIO, r2p_sub_node, NULL);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(4096), NORMALPRIO, rosserial_pub_thread, NULL);
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO, rosserial_sub_thread, NULL);

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
}

