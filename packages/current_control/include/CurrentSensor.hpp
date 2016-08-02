#pragma once

#include <current_control/CurrentSensorConfiguration.hpp>
#include <Core/MW/CoreSensor.hpp>

#include "ch.h"
#include "hal.h"

#define ADC_NUM_CHANNELS   1
#define ADC_BUF_DEPTH      1

namespace current_control {

class CurrentSensor: public Core::MW::CoreSensor<float> {
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

	void
	get(DataType& data);

public:
	static void
	setCallback(std::function<void(DataType)>& callback);

	static void
	current_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n);

public:
	static CurrentSensorConfiguration configuration;

private:
	static std::function<void(DataType)> adcCallback;
	static adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];
	static bool onCycle;
	static float currentPeakHigh;
	static float currentPeakLow;

protected:
	Core::MW::Time _timestamp;
	DataType _data;

};

}
