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
/* Motor calibration.                                                        */
/*===========================================================================*/

static int pwm = 0;

#define ADC_NUM_CHANNELS  1
#define ADC_BUF_DEPTH     1

static float meanLevel = 0.0f;

static adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

static void current_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n);

/*
 * ADC conversion group.
 * Mode:        Circular buffer, 1 sample of 1 channel, triggered by pwm channel 3
 * Channels:    IN10.
 */
static const ADCConversionGroup adcgrpcfg = { TRUE, // circular
		ADC_NUM_CHANNELS, // num channels
		current_callback, // end callback
		NULL, // error callback
		0, // CR1
		ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_1, // CR2
		0, // SMPR1
		ADC_SMPR2_SMP_AN3(ADC_SAMPLE_1P5)/*ADC_SMPR2_SMP_AN3(ADC_SAMPLE_239P5)*/, // SMPR2
		ADC_SQR1_NUM_CH(ADC_NUM_CHANNELS), // SQR1
		0, // SQR2
		ADC_SQR3_SQ1_N(ADC_CHANNEL_IN3) // SQR3
		};

static void current_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

	(void) adcp;
	(void) n;

	palTogglePad(LED1_GPIO, LED1);

	chSysLockFromIsr()
	;

	meanLevel = buffer[0];

	//Compute current
	if (tp_motor != NULL) {
		chSchReadyI(tp_motor);
		tp_motor = NULL;
	}

	chSysUnlockFromIsr();

}


static PWMConfig pwmcfg = { STM32_SYSCLK, // 72MHz PWM clock frequency.
		4096, // 12-bit PWM, 17KHz frequency.
		NULL, // pwm callback
		{
				{ PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL }, //
				{ PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL }, //
				{ PWM_OUTPUT_ACTIVE_LOW, NULL }, //
				{ PWM_OUTPUT_DISABLED, NULL } }, //
				0, //
#if STM32_PWM_USE_ADVANCED
		72, /* XXX 1uS deadtime insertion   */
#endif
		0 };

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

	// Start the ADC driver and conversion
	adcStart(&ADC_DRIVER, NULL);
	adcStartConversion(&ADC_DRIVER, &adcgrpcfg, adc_samples, ADC_BUF_DEPTH);

	// Init motor driver
	palSetPad(DRIVER_GPIO, DRIVER_RESET);
	chThdSleepMilliseconds(500);
	pwmStart(&PWM_DRIVER, &pwmcfg);

	// wait some time
	chThdSleepMilliseconds(500);

	// start pwm
	float voltage = -12.0;

	const float pwm_res = 4095.0f / 24.0f;
	pwm = static_cast<int>(voltage * pwm_res);

	if (pwm > 0) {
		pwm_lld_enable_channel(&PWM_DRIVER, 1, pwm);
		pwm_lld_enable_channel(&PWM_DRIVER, 0, 0);

		pwm_lld_enable_channel(&PWM_DRIVER, 2, pwm/2);
	} else {
		pwm_lld_enable_channel(&PWM_DRIVER, 1, 0);
		pwm_lld_enable_channel(&PWM_DRIVER, 0, -pwm);

		pwm_lld_enable_channel(&PWM_DRIVER, 2, -pwm/2);
	}

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
