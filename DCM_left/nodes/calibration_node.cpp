/*
 * calibration_node.cpp
 *
 *  Created on: 02/nov/2015
 *      Author: dave
 */

#include "ch.h"
#include "hal.h"

#include "r2p/Middleware.hpp"

#include "calibration_node.hpp"

static Thread *tp_motor = NULL;

namespace r2p {

/*===========================================================================*/
/* Bufferizing node.                                                         */
/*===========================================================================*/

static calibration_node_conf defaultConf = { "calibration_node", "bits",
		"bits_packed" };

msg_t calibration_node(void* arg) {
	calibration_node_conf* conf;
	if (arg != NULL)
		conf = (calibration_node_conf *) arg;
	else
		conf = &defaultConf;

	Node node(conf->name);

	Subscriber<FloatMsg, 5> calibration_sub;
	FloatMsg * msgp_in;

	Publisher<FloatPackMsg> calibration_pub;
	FloatPackMsg * msgp_out;

	chRegSetThreadName(conf->name);

	node.subscribe(calibration_sub, conf->topicIn);
	node.advertise(calibration_pub, conf->topicOut);

	int count = 0;
	int buffer[20];

	for (;;) {

		if (node.spin(r2p::Time::ms(1000))) {

			// fetch data
			if (calibration_sub.fetch(msgp_in)) {
				buffer[count++] = msgp_in->value;
				calibration_sub.release(*msgp_in);
			}

			// publish data in batches
			if (count == 20) {
				if (calibration_pub.alloc(msgp_out)) {

					for (int i = 0; i < 20; i++) {
						msgp_out->value[i] = buffer[i];
					}

					calibration_pub.publish(*msgp_out);
				}

				count = 0;
			}
		}

	}

}

/*===========================================================================*/
/* Motor calibration.                                                        */
/*===========================================================================*/

#define ADC_NUM_CHANNELS   1
#define ADC_BUF_DEPTH      128

static float meanLevel = 0.0f;

static adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

static void current_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

	(void) adcp;

	//Compute current
	int levels = 0;
	for (unsigned int i = 0; i < n; i++) {
		levels += buffer[i];
	}

	meanLevel = (float) levels / (float) n;

	chSysLockFromIsr()
	;
	if (tp_motor != NULL) {
		chSchReadyI(tp_motor);
		tp_motor = NULL;
	}
	chSysUnlockFromIsr();

}

/*
 * PWM configuration.
 */
static PWMConfig pwmcfg = { STM32_SYSCLK, /* 72MHz PWM clock frequency.   */
4096, /* 12-bit PWM, 17KHz frequency. */
NULL, { { PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL },
		{ PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL },
		{ PWM_OUTPUT_DISABLED, NULL }, { PWM_OUTPUT_DISABLED, NULL } }, 0,
#if STM32_PWM_USE_ADVANCED
		72, /* XXX 1uS deadtime insertion   */
#endif
		0 };

/*
 * ADC conversion group.
 * Mode:        Circular buffer, 8 samples of 1 channel.
 * Channels:    IN10.
 */
static const ADCConversionGroup adcgrpcfg = {
TRUE,
ADC_NUM_CHANNELS, current_callback,
NULL, 0, 0, /* CR1, CR2 */
0, ADC_SMPR2_SMP_AN3(ADC_SAMPLE_239P5), /* SMPR2 */
ADC_SQR1_NUM_CH(ADC_NUM_CHANNELS), 0, /* SQR2 */
ADC_SQR3_SQ1_N(ADC_CHANNEL_IN3) };

static calibration_pub_node_conf defaultPubConf = { "motor_calibration_node",
		"bits" };

msg_t motor_calibration_node(void * arg) {
	//Configure current node
	calibration_pub_node_conf* conf;
	if (arg != NULL)
		conf = (calibration_pub_node_conf *) arg;
	else
		conf = &defaultPubConf;

	Node node(conf->name);
	Publisher<FloatMsg> current_pub;
	FloatMsg * msgp;

	chRegSetThreadName(conf->name);

	node.advertise(current_pub, conf->topic);

	// Init motor driver
	palSetPad(DRIVER_GPIO, DRIVER_RESET);
	chThdSleepMilliseconds(500);
	pwmStart(&PWM_DRIVER, &pwmcfg);

	// start pwm
	int pwm = 2048;
	pwm_lld_enable_channel(&PWM_DRIVER, 1, pwm);
	pwm_lld_enable_channel(&PWM_DRIVER, 0, 0);

	// wait some time
	chThdSleepMilliseconds(500);

	// Start the ADC driver and conversion
	adcStart(&ADC_DRIVER, NULL);
	adcStartConversion(&ADC_DRIVER, &adcgrpcfg, adc_samples, ADC_BUF_DEPTH);

	// Start publishing current measures
	for (;;) {
		// Wait for interrupt
		chSysLock()
		;
		tp_motor = chThdSelf();
		chSchGoSleepS(THD_STATE_SUSPENDED);
		chSysUnlock();

		// publish current
		if (current_pub.alloc(msgp)) {
			msgp->value = meanLevel;
			current_pub.publish(*msgp);
		}

	}

	return CH_SUCCESS;
}

}
