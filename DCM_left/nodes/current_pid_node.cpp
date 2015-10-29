#include "ch.h"
#include "hal.h"

#include "r2p/Middleware.hpp"

#include "current_pid_node.hpp"

#include <r2p/node/pid.hpp>

#include "config.h"

namespace r2p {

#define M1 72
#define M2 73
#define M3 89

/*===========================================================================*/
/* Motor parameters.                                                         */
/*===========================================================================*/
#define ADC_NUM_CHANNELS   1
#define ADC_BUF_DEPTH      32

//#define _R                 0.117f
#define _R                 8.5f
//#define _L                 2.5e-5f
#define _L                 1.17e-3f

#define _Ts                (252.0/72.0e6*(float)ADC_BUF_DEPTH)

#define _pwmTicks          4096.0f

static PID current_pid;
static int pwm = 0;
static float maxV = 0;

/*===========================================================================*/
/* Current sensor parameters.                                                */
/*===========================================================================*/

#define _Kcs              -0.0074
#define _Qcs               15.1295


/*===========================================================================*/
/* Current sense related.                                                    */
/*===========================================================================*/

adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

void current_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

	chSysLock()
	;

	//Compute current
	int levels = 0.0;
	for(unsigned int i = 0; i < n; i++)
	{
		levels += buffer[i];
	}

	float meanLevel = (float) levels / (float) n;

	float current = meanLevel * _Kcs + _Qcs ;

	//compute control signal
	float voltage = current_pid.update(current);

	//Compute pwm signal and apply
	pwm = _pwmTicks/maxV*voltage;

	if (pwm > 0) {
		pwm_lld_enable_channel(&PWM_DRIVER, 1, pwm);
		pwm_lld_enable_channel(&PWM_DRIVER, 0, 0);
	} else {
		pwm_lld_enable_channel(&PWM_DRIVER, 1, 0);
		pwm_lld_enable_channel(&PWM_DRIVER, 0, -pwm);
	}
	chSysUnlock();

	palTogglePad(LED2_GPIO, LED2);

}

/*
 * ADC conversion group.
 * Mode:        Circular buffer, 8 samples of 1 channel.
 * Channels:    IN10.
 */
static const ADCConversionGroup adcgrpcfg = {
TRUE,
ADC_NUM_CHANNELS,
current_callback,
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

current_pid_node_conf defaultConf =
{
	"current_pid",
	0,
	0.110f,
	1.17e-3f,
	6000.0f,
	24.0f
};

msg_t current_pid2_node(void * arg) {
	//Configure current node
	current_pid_node_conf* conf;
	if(arg != NULL)
		conf = (current_pid_node_conf *) arg;
	else
		conf = &defaultConf;


	Node node(conf->name);
	Subscriber<Current2Msg, 5> current_sub;
	Current2Msg * msgp;
	Time last_setpoint(0);

	(void) arg;

	chRegSetThreadName(conf->name);

	float Kp = conf->omegaC*conf->L;
	float Ti = conf->L/conf->R;
	maxV = conf->maxV;
	current_pid.config(Kp, Ti, 0.0, _Ts, -maxV, maxV);

	// Init motor driver
	palSetPad(DRIVER_GPIO, DRIVER_RESET);
	chThdSleepMilliseconds(500);
	pwmStart(&PWM_DRIVER, &pwmcfg);

	// Start the ADC driver and conversion
	adcStart(&ADC_DRIVER, NULL);
	adcStartConversion(&ADC_DRIVER, &adcgrpcfg, adc_samples, ADC_BUF_DEPTH);


	/*node.subscribe(current_sub, "current2");

	for (;;) {
		if (node.spin(Time::ms(100))) {
			if (current_sub.fetch(msgp)) {
				current_pid.set(msgp->value[index]);
				last_setpoint = Time::now();
				current_sub.release(*msgp);

				palTogglePad(LED3_GPIO, LED3);

			} else if (Time::now() - last_setpoint > Time::ms(100)) {
				current_pid.set(0);
			}
		} else {
			// Stop motor if no messages for 100 ms
			pwm_lld_disable_channel(&PWM_DRIVER, 0);
			pwm_lld_disable_channel(&PWM_DRIVER, 1);

			palTogglePad(LED4_GPIO, LED4);
		}
	}*/

	current_pid.set(-1.0);

	for (;;) {
		if (node.spin(Time::ms(100))) {
			palTogglePad(LED3_GPIO, LED3);

		}
		else {
			palTogglePad(LED4_GPIO, LED4);
		}
	}

	return CH_SUCCESS;
}

} /* namespace r2p */
