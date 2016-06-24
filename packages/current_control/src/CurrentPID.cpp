#include <current_control/CurrentPID.hpp>
#include <pid_ie/pid_ie.hpp>

#include "ch.h"
#include "hal.h"

//static thread_t *tp_motor = NULL;

namespace current_control
{

/*===========================================================================*/
/* Macro definitions.                                                        */
/*===========================================================================*/

#define ABS(x) ((x) >= 0) ? (x) : -(x)

/*===========================================================================*/
/* Motor parameters.                                                         */
/*===========================================================================*/
#define ADC_NUM_CHANNELS   1
#define ADC_BUF_DEPTH      1

#define _Ts                (1.0f/17.5e3)
#define _pwmTicks          4096.0f
#define _pwmMin            16
#define _controlCycles     1

static PID_IE current_pid;
static float currentPeak = 0.0f;
static float current = 0.0f;
static float measure = 0.0f;

static float Kpwm;
static int pwm = 0;
static int controlCounter = 0;

/*===========================================================================*/
/* Current sensor parameters.                                                */
/*===========================================================================*/

#define _Kcs               -0.007179470922790f
#define _Qcs               14.637146343837731f

/*===========================================================================*/
/* Config adc and pwm                                                        */
/*===========================================================================*/

static adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

static void current_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

	(void) adcp;
	(void) n;

	//Compute current
	chSysLockFromISR();
	currentPeak = (_Kcs * buffer[0] + _Qcs);
	palTogglePad(GPIOA, GPIOA_ENCODER1_A); //TODO Levami!!!
	chSysUnlockFromISR();

}

static void control_callback(PWMDriver *pwmp) {
	(void) pwmp;

	chSysLockFromISR();

		//Add new current peak
	current += currentPeak;

	//Count cycle
	controlCounter++;

	//compute control if control cycle
	if (controlCounter == _controlCycles) {

		// Compute mean current
		current /= _controlCycles;

		// Compute control
		float voltage = current_pid.update(current);

		//Compute pwm signal
		pwm = Kpwm * voltage;

		//Set pwm to 0 if not in controllable region (also current peak that will not be updated)
		int dutyCycle = ABS(pwm);
		if (dutyCycle <= _pwmMin) {
			pwm = 0;
			currentPeak = 0;
			dutyCycle = 0;
		}

		pwm_lld_enable_channel(&PWM_DRIVER, pwm > 0 ? 1 : 0, dutyCycle);
		pwm_lld_enable_channel(&PWM_DRIVER, pwm > 0 ? 0 : 1, 0);

		pwm_lld_enable_channel(&PWM_DRIVER, 2, dutyCycle / 2);

		//set measure
		measure = current;

		// reset variables
		current = 0;
		controlCounter = 0;
	}

	//Wake up thread
	/*if (tp_motor != NULL) {
		chSchReadyI(tp_motor);
		tp_motor = NULL;
	}*/
	chSysUnlockFromISR();

}

static const ADCConversionGroup adcgrpcfg = {
  TRUE, // circular
  ADC_NUM_CHANNELS, // num channels
  current_callback, // end callback
  NULL, // error callback
  ADC_CFGR_EXTEN_RISING | ADC_CFGR_EXTSEL_SRC(9), // CFGR
  ADC_TR(0, 4095),                                // TR1
  ADC_CCR_DUAL(1),                                // CCR
  {                                               // SMPR[2]
  	0,
  	0
  },
  {                                 // SQR[4]
      0,
      0,
      0,
      0
  }/*,
  {                                               // SMPR[2]
	0,
	ADC_SMPR2_SMP_AN12(ADC_SMPR_SMP_1P5)
  },
  {                                 // SQR[4]
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN12),
    0,
    0,
    0
  }*/
};

/*===========================================================================*/
/* Motor control nodes.                                                      */
/*===========================================================================*/

/*
 * PID node.
 */

CurrentPID::CurrentPID(const char* name,
					   Core::MW::CoreActuator<float>& pwm,
					   Core::MW::Thread::PriorityEnum priority) :
      CoreNode::CoreNode(name, priority), _pwm(pwm)
   {
      _workingAreaSize = 512;
   }

CurrentPID::~CurrentPID()
{
    teardown();
}

bool
CurrentPID::onConfigure()
{
    //Set PWM gain
	Kpwm = 1.0 / configuration.maxV;
    
    //Compute PID tuning
    const float Kp = configuration.omegaC * configuration.L;
	const float Ti = configuration.L / configuration.R;
    
    //tune PID
	current_pid.config(Kp, Ti, 0.0, _Ts, 0.0, -configuration.maxV, configuration.maxV);
    
    //set pid setpoint
	current_pid.set(0.0);
    
    return true;
}

bool
CurrentPID::onPrepareHW()
{
	//TODO implement
    // Start the ADC driver and conversion
	ADCDriver* ADC_DRIVER = &ADCD1;

	adcStart(ADC_DRIVER, NULL);
	adcStartConversion(ADC_DRIVER, &adcgrpcfg, adc_samples, ADC_BUF_DEPTH);

	palSetPadMode(GPIOA, GPIOA_ENCODER1_A, PAL_MODE_OUTPUT_PUSHPULL);
	_pwm.start();

	float value = 0.5;

	_pwm.set(value);

    return true;
}

bool
CurrentPID::onPrepareMW()
{
    this->subscribe(_subscriber, "current2");
    


    return true;
}

bool
CurrentPID::onLoop()
{

	extra_msgs::Current2 * msgp_in;

	Core::MW::Time last_setpoint(0);


	uint32_t index = configuration.index;

	this->spin(Core::MW::Time::ms(100));

	// read setpoint TODO implement
	/*for (;;) {

		if (this->spin(Core::MW::Time::ms(100)) && _subscriber.fetch(msgp_in)) {
			chSysLock();
			current_pid.set(msgp_in->value[index]);
			chSysUnlock();
			last_setpoint = Core::MW::Time::now();
			_subscriber.release(*msgp_in);

		} else if (Core::MW::Time::now() - last_setpoint > Core::MW::Time::ms(100)) {
			chSysLock();
			current_pid.reset();
			chSysUnlock();
		}

	}*/


    return true;
}

/*static current_publisher_node_conf defaultPubConf = { "current_publisher",
		"current_measure" };

msg_t current_publisher_node(void * arg) {
	//Configure current node
	current_publisher_node_conf* conf;
	if (arg != NULL)
		conf = (current_publisher_node_conf *) arg;
	else
		conf = &defaultPubConf;

	Node node(conf->name);
	Publisher<CurrentMsg> current_pub;

	CurrentMsg * msgp_out;

	chRegSetThreadName(conf->name);

	node.advertise(current_pub, conf->topic);

	for (;;) {
		// Wait for interrupt
		chSysLock()
		tp_motor = chThdSelf();
		chSchGoSleepS(THD_STATE_SUSPENDED);
		chSysUnlock();

		// publish current
		if (current_pub.alloc(msgp_out)) {
			chSysLock()
			msgp_out->value = measure;
			chSysUnlock();
			current_pub.publish(*msgp_out);
		}

	}

	return CH_SUCCESS;
}*/

} /* namespace current_control */
