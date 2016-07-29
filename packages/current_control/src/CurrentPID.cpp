#include <current_control/CurrentPID.hpp>

#include "ch.h"
#include "hal.h"

using namespace std::placeholders;

namespace current_control
{

/*===========================================================================*/
/* Current sensor parameters.                                                */
/*===========================================================================*/

#define _Kcs               0.007432946790511f
#define _Qcs               -15.207809133385506f

/*===========================================================================*/
/* Config adc					                                             */
/*===========================================================================*/

#define ADC_NUM_CHANNELS   1
#define ADC_BUF_DEPTH      1

static bool onCycle = true;
static float currentPeakHigh = 0.0f;
static float currentPeakLow = 0.0f;

static adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

static std::function<void(float)> adcCallback;

static void current_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

	(void) adcp;
	(void) n;

	//Compute current
	chSysLockFromISR();
	if(onCycle)
	{
		currentPeakHigh = (_Kcs * buffer[0] + _Qcs);
		if(adcCallback)
			adcCallback(currentPeakHigh);
	}
	else
		currentPeakLow = (_Kcs * buffer[0] + _Qcs);

	onCycle = !onCycle;

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
  	ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_1P5),
  	0
  },
  {                                              // SQR[4]
	  ADC_SQR1_SQ1_N(ADC_CHANNEL_IN1),
      0,
      0,
      0
  }
};


/*===========================================================================*/
/* Motor control nodes.                                                      */
/*===========================================================================*/

CurrentPID::CurrentPID(const char* name,
					   Core::MW::CoreActuator<float>& pwm,
					   Core::MW::Thread::PriorityEnum priority) :
      CoreNode::CoreNode(name, priority), _pwm(pwm)
   {
	  _Kpwm = 0.0f;
	  _current = 0.0f;
	  _controlCounter = 0;
	  _controlCycles = 0;
      _workingAreaSize = 256;
   }

CurrentPID::~CurrentPID()
{
    teardown();
}

void CurrentPID::controlCallback(float currentPeak)
{
	//Add new current peak
	_current += currentPeak;

	//Count cycle
	_controlCounter++;

	//compute control if control cycle
	if (_controlCounter == _controlCycles) {

		// Compute mean current
		_current /= _controlCycles;

		// Compute control
		float voltage = _currentPID.update(_current);

		//Compute pwm signal
		float pwm = _Kpwm * voltage;
		_pwm.set(pwm);

		// reset variables
		_current = 0;
		_controlCounter = 0;
	}

}

bool
CurrentPID::onConfigure()
{
    //Set PWM gain
	_Kpwm = 1.0 / configuration.maxV;
	_controlCycles = configuration.controlCycles;

    
    //Compute PID tuning
    const float Kp = configuration.omegaC * configuration.L;
	const float Ti = configuration.L / configuration.R;
    const float Ts = _controlCycles/17.5e3; //TODO read from pwm driver

    //tune PID
	_currentPID.config(Kp, Ti, 0.0f, Ts, 0.0f, -configuration.maxV, configuration.maxV);
    
    //set pid setpoint
	_currentPID.set(0.0);
    
    return true;
}

bool
CurrentPID::onPrepareHW()
{
    // Start the ADC driver and conversion
	ADCDriver* ADC_DRIVER = &ADCD3;

	adcStart(ADC_DRIVER, NULL);
	adcStartConversion(ADC_DRIVER, &adcgrpcfg, adc_samples, ADC_BUF_DEPTH);

	adcCallback = std::bind(&CurrentPID::controlCallback, this, _1);

	//Start pwm
	_pwm.start();

	float value = 0.0f;
	_pwm.set(value);

    return true;
}

bool
CurrentPID::onPrepareMW()
{
    subscribe(_subscriber, configuration.topic);
    _subscriber.set_callback(CurrentPID::callback);

    return true;
}

bool
CurrentPID::callback(
   const actuator_msgs::Setpoint_f32& msg,
   Core::MW::Node*                    node
)
{
   CurrentPID* _this = static_cast<CurrentPID*>(node);

   chSysLock();
   _this->_currentPID.set(msg.value);
   chSysUnlock();

   return true;
}



bool
CurrentPID::onLoop()
{
	if (!this->spin(Core::MW::Time::ms(100)))
	{
		chSysLock();
		_currentPID.reset();
		chSysUnlock();
	}

    return true;
}

} /* namespace current_control */
