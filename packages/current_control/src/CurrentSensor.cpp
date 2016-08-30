#include <core/current_control/CurrentSensor.hpp>

namespace core {

namespace current_control {

/*===========================================================================*/
/* Config adc					                                             */
/*===========================================================================*/

CurrentSensorConfiguration CurrentSensor::configuration;
std::function<void(CurrentSensor::DataType, CurrentSensor::DataType)> CurrentSensor::adcCallback;
adcsample_t CurrentSensor::adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];
bool CurrentSensor::onCycle;
float CurrentSensor::currentPeakHigh;
float CurrentSensor::currentPeakLow;
float CurrentSensor::Vcc;

void
CurrentSensor::setCallback(std::function<void(DataType, DataType)>& callback)
{
	adcCallback = callback;
}

void CurrentSensor::current_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

	(void) adcp;
	(void) n;

	//Compute current
	chSysLockFromISR();

	Vcc = (4.7+39.0)*3.3/4095.0/4.7*buffer[1];

	if(onCycle)
	{
		currentPeakHigh = (configuration.a * buffer[0] + configuration.b);

		if(adcCallback)
			adcCallback(currentPeakHigh, Vcc);
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
  nullptr, // error callback
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
  },
#ifdef STM32_ADC_DUAL_MODE
  {                                               // SMPR[2]
	   ADC_SMPR1_SMP_AN3(ADC_SMPR_SMP_1P5),
       0
  },
  {                                              // SQR[4]
       ADC_SQR1_SQ1_N(ADC_CHANNEL_IN3),
       0,
       0,
       0
  },

#endif
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
	chSysLock();
	_data = currentPeakHigh;
	chSysUnlock();
	return true;
}

bool CurrentSensor::configure()
{
	return true;
}

void CurrentSensor::get(DataType& data)
{
	data = _data;
}

}
}
