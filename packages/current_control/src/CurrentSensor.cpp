#include <current_control/CurrentSensor.hpp>

namespace current_control {

/*===========================================================================*/
/* Config adc					                                             */
/*===========================================================================*/

CurrentSensorConfiguration CurrentSensor::configuration;
std::function<void(CurrentSensor::DataType)> CurrentSensor::adcCallback;
adcsample_t CurrentSensor::adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];
bool CurrentSensor::onCycle;
float CurrentSensor::currentPeakHigh;
float CurrentSensor::currentPeakLow;


void
CurrentSensor::setCallback(std::function<void(DataType)>& callback)
{
	adcCallback = callback;
}

void CurrentSensor::current_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

	(void) adcp;
	(void) n;

	//Compute current
	chSysLockFromISR();
	if(onCycle)
	{
		currentPeakHigh = (configuration.a * buffer[0] + configuration.b);
		if(adcCallback)
			adcCallback(currentPeakHigh);
	}
	else
		currentPeakLow = (configuration.a * buffer[0] + configuration.b);

	onCycle = !onCycle;

	chSysUnlockFromISR();

}

static const ADCConversionGroup adcgrpcfg = {
  TRUE, // circular
  ADC_NUM_CHANNELS, // num channels
  CurrentSensor::current_callback, // end callback
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


CurrentSensor::CurrentSensor()
{
	_data = 0;
}

CurrentSensor::~CurrentSensor()
{

}

bool CurrentSensor::init()
{
	onCycle = true;
	currentPeakHigh = 0.0f;
	currentPeakLow = 0.0f;

	return true;
}

bool CurrentSensor::start()
{
    // Start the ADC driver and conversion
	adcStart(&ADCD3, NULL);
	adcStartConversion(&ADCD3, &adcgrpcfg, adc_samples, ADC_BUF_DEPTH);

	return true;
}

bool CurrentSensor::stop()
{
	adcStop(&ADCD3);

	return true;
}

bool CurrentSensor::waitUntilReady()
{
	//TODO implement
	return true;
}

bool CurrentSensor::update()
{
	//TODO implement correctly
	_data = currentPeakHigh;
	return true;
}

void CurrentSensor::get(DataType& data)
{
	data = _data;
}

}
