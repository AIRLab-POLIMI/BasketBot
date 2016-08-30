#pragma once

#include <core/current_control/CurrentSensorConfiguration.hpp>
#include <core/mw/CoreSensor.hpp>

#include "ch.h"
#include "hal.h"

#define ADC_NUM_CHANNELS   2
#define ADC_BUF_DEPTH      1

namespace core {

namespace current_control {

class CurrentSensor: public core::mw::CoreSensor<float> {
public:
	CurrentSensor();

	virtual
	~CurrentSensor();

public:
	bool
	init();

	bool
	start();

	bool
	stop();

	bool
	waitUntilReady();

	bool
	update();

	bool
	configure();

	void
	get(DataType& data);

public:
	static void
	setCallback(std::function<void(DataType, DataType)>& callback);

	static void
	current_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n);

public:
	static CurrentSensorConfiguration configuration;

private:
	static std::function<void(DataType, DataType)> adcCallback;
	static adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];
	static bool onCycle;
	static float currentPeakHigh;
	static float currentPeakLow;
	static float Vcc;

protected:
	core::os::Time _timestamp;
	DataType _data;

};

}

}
