#ifndef _PID_EI_EI_HPP_
#define _PID_EI_EI_HPP_

#include <float.h>

class PID_IE {
private:
	float _ui;
	float _ud;
	float _errorOld;
	float _setpoint;
	float _k;
	float _a;
	float _b1;
	float _b2;
	float _min;
	float _max;

	bool _auto;
	bool _saturation;
	float _control;

public:
	PID_IE(void);
	void config(float k, float ti, float td, float ts, float N, float min,
			float max);
	void set(float setpoint);
	void reset(void);
	float get_setpoint(void);
	float update(float measure);

	void setAuto(bool autoMode);
	void setControl(float control);

	void setSaturation(bool saturation);
};

PID_IE::PID_IE(void) {
	_ui = 0;
	_ud = 0;
	_errorOld = 0;
	_setpoint = 0;
	_k = 0;
	_a = 0;
	_b1 = 0;
	_b2 = 0;
	_min = -FLT_MAX;
	_max = FLT_MAX;

	_auto = true;
	_saturation = false;
	_control = 0;

}

void PID_IE::config(float k, float ti, float td, float ts, float N, float min =
		-FLT_MAX, float max = FLT_MAX) {

	_k = k;
	_a = (ti == 0) ? 0 : k * (ts / ti);
	_b1 = (td == 0) ? 0 : td / (N * ts + td);
	_b2 = (td == 0) ? 0 : k * N * _b1;
	_min = min;
	_max = max;
}

void PID_IE::set(float setpoint) {

	_setpoint = setpoint;
}

void PID_IE::reset(void) {

	_ui = 0;
	_ud = 0;
	_errorOld = 0;
	_setpoint = 0;

	_auto = true;
	_saturation = false;
	_control = 0;
}

float PID_IE::get_setpoint(void) {
	return _setpoint;
}

float PID_IE::update(float measure) {
	float error;
	float output;

	/* calculate error */
	error = _setpoint - measure;

	/* proportional term */
	float up = _k * error;

	/* derivative term */
	_ud = _b1 * _ud + _b2 * (error - _errorOld);

	/* compute control */
	if (_auto) {
		output = up + _ui + _ud;
	} else {
		output = _control;
	}

	/* saturation filter */
	if (output > _max) {
		output = _max;
	} else if (output < _min) {
		output = _min;
	} else if (!_saturation) {
		if (_auto) {
			/* integral term - auto */
			_ui = _ui + _a * error;
		} else {
			/* integral term - manual */
			_ui = output - up - _ud;
		}
	}

	_errorOld = error;

	return output;
}

void PID_IE::setAuto(bool autoMode) {
	_auto = autoMode;
}

void PID_IE::setControl(float control) {
	_control = control;
}

void PID_IE::setSaturation(bool saturation) {
	_saturation = saturation;
}

#endif /* _PID_EI_EI_HPP_ */
