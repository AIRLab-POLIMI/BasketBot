#include "ch.h"
#include "hal.h"

#include "r2p/Middleware.hpp"

#include "current_pid_node.hpp"

#include <r2p/node/pid.hpp>

#include "config.h"

static Thread *tp = NULL;

namespace r2p {

#define M1 72
#define M2 73
#define M3 89

/*===========================================================================*/
/* Motor parameters.                                                         */
/*===========================================================================*/
#define ADC_NUM_CHANNELS   1
#define ADC_BUF_DEPTH      128

#define _Ts                (252.0/72.0e6*(float)ADC_BUF_DEPTH/2.0)
#define _pwmTicks          4096.0f

static PID current_pid;
static float Kpwm = 0.0f;
static float meanLevel = 0.0f;

/*===========================================================================*/
/* Current sensor parameters.                                                */
/*===========================================================================*/

#define _Kcs               -0.0074f
#define _Qcs               +15.1295f

/*===========================================================================*/
/* Current sense related.                                                    */
/*===========================================================================*/

adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

void current_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

	(void) adcp;

	//Compute current
	int levels = 0;
	for (unsigned int i = 0; i < n; i++) {
		levels += buffer[i];
	}

	meanLevel = (float) levels / (float) n;

	chSysLockFromIsr()
	;
	if (tp != NULL) {
		chSchReadyI(tp);
		tp = NULL;
	}
	chSysUnlockFromIsr();

}

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

/*===========================================================================*/
/* Motor control nodes.                                                      */
/*===========================================================================*/

/*
 * PID node.
 */

static current_pid_node_conf defaultConf = { "current_pid", "current_measure", 0,
		0.110f, 2.5e-5f, 6000.0f, 24.0f };

msg_t current_pid2_node(void * arg) {
	//Configure current node
	current_pid_node_conf* conf;
	if (arg != NULL)
		conf = (current_pid_node_conf *) arg;
	else
		conf = &defaultConf;

	Node node(conf->name);
	Publisher<CurrentMsg> current_pub;
	Subscriber<Current2Msg, 5> current_sub;
	Current2Msg * msgp_in;
	CurrentMsg * msgp_out;

	Time last_setpoint(0);

	chRegSetThreadName(conf->name);

	int index = conf->index;
	float Kp = conf->omegaC * conf->L;
	float Ti = conf->L / conf->R;
	Kpwm = _pwmTicks / conf->maxV;
	current_pid.config(Kp, Ti, 0.0, _Ts, -conf->maxV, conf->maxV);

	// Init motor driver
	palSetPad(DRIVER_GPIO, DRIVER_RESET);
	chThdSleepMilliseconds(500);
	pwmStart(&PWM_DRIVER, &pwmcfg);

	// Start the ADC driver and conversion
	adcStart(&ADC_DRIVER, NULL);
	adcStartConversion(&ADC_DRIVER, &adcgrpcfg, adc_samples, ADC_BUF_DEPTH);

	node.subscribe(current_sub, "current2");
	node.advertise(current_pub, conf->topic);

	current_pid.set(0.0);

	for (;;) {
		// Wait for interrupt
		chSysLock()
		;
		tp = chThdSelf();
		chSchGoSleepS(THD_STATE_SUSPENDED);
		chSysUnlock();

		//compute current
		float current = meanLevel * _Kcs + _Qcs;

		//compute control signal
		float voltage = current_pid.update(current);

		//Compute pwm signal and apply
		int pwm = Kpwm * voltage;

		if (pwm > 0) {
			pwm_lld_enable_channel(&PWM_DRIVER, 1, pwm);
			pwm_lld_enable_channel(&PWM_DRIVER, 0, 0);
		} else {
			pwm_lld_enable_channel(&PWM_DRIVER, 1, 0);
			pwm_lld_enable_channel(&PWM_DRIVER, 0, -pwm);
		}

		palTogglePad(LED2_GPIO, LED2);

		// update setpoint
		if (current_sub.fetch(msgp_in)) {
			current_pid.set(msgp_in->value[index]);
			last_setpoint = Time::now();
			current_sub.release(*msgp_in);

			palTogglePad(LED3_GPIO, LED3);

		} else if (Time::now() - last_setpoint > Time::ms(100)) {
			current_pid.set(2.5);
			palTogglePad(LED4_GPIO, LED4);
		}

		// publish current
		if (current_pub.alloc(msgp_out)) {
			msgp_out->value = current;
			current_pub.publish(*msgp_out);
		}

	}

	return CH_SUCCESS;
}

} /* namespace r2p */
