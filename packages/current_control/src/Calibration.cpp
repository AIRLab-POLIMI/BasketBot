#include <current_control/CurrentPID.hpp>

#include "ch.h"
#include "hal.h"

namespace current_control
{


/*===========================================================================*/
/* Config adc					                                             */
/*===========================================================================*/

#define ADC_NUM_CHANNELS   1
#define ADC_BUF_DEPTH      1

static bool onCycle = true;
static unsigned int currentPeakHigh = 0.0f;
static unsigned int currentPeakLow = 0.0f;

static adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

std::function<void()> adcCallback;

static void current_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

	(void) adcp;
	(void) n;

	//Compute current
	chSysLockFromISR();
	if(onCycle)
	{
		currentPeakHigh = buffer[0];
		if(adcCallback)
			adcCallback();
	}
	else
		currentPeakLow = buffer[0];

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

Calibration::Calibration(const char* name,
					   Core::MW::CoreActuator<float>& pwm,
					   Core::MW::Thread::PriorityEnum priority) :
      CoreNode::CoreNode(name, priority), _pwm(pwm)
   {
	  _current = 0.0f;
      _workingAreaSize = 512;
   }

Calibration::~Calibration()
{
    teardown();
}

void Calibration::calibrationCallback()
{
	chSysLockFromISR();

	//Add new current peak
	_current += currentPeakHigh;

	chSysUnlockFromISR();

}


bool
Calibration::onPrepareHW()
{
    // Start the ADC driver and conversion
	ADCDriver* ADC_DRIVER = &ADCD3;

	adcStart(ADC_DRIVER, NULL);
	adcStartConversion(ADC_DRIVER, &adcgrpcfg, adc_samples, ADC_BUF_DEPTH);

	adcCallback = std::bind(&CurrentPID::controlCallback, this);

	palSetPadMode(GPIOA, GPIOA_ENCODER1_A, PAL_MODE_OUTPUT_PUSHPULL);

	//Start pwm
	_pwm.start();
	float value = 1.0;
	_pwm.set(value);

    return true;
}


bool
Calibration::onLoop()
{
	this->spin(Core::MW::Time::s(100));

    return true;
}

} /* namespace current_control */
