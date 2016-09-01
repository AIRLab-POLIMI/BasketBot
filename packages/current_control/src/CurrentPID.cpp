#include <core/current_control/CurrentPID.hpp>

#include <cmath>

using namespace std::placeholders;


namespace core
{
namespace current_control
{

/*===========================================================================*/
/* Motor control nodes.                                                      */
/*===========================================================================*/

CurrentPID::CurrentPID(const char* name,
					   CurrentSensor& currentSensor,
					   core::mw::CoreActuator<float>& pwm,
					   core::os::Thread::PriorityEnum priority) :
      CoreNode::CoreNode(name, priority),
	  core::mw::CoreConfigurable<core::current_control::CurrentPIDConfiguration>(name),
	  _currentSensor(currentSensor), _pwm(pwm)
   {
      _sign = 0;
	  _Kpwm = 0.0f;
	  _Ktorque = 0.0f;
	  _current = 0.0f;
	  _controlCounter = 0;
	  _controlCycles = 0;
	  _underVoltage = false;

      _workingAreaSize = 256;
   }

CurrentPID::~CurrentPID()
{
    teardown();
}

void CurrentPID::controlCallback(float currentPeak, float Vcc)
{

	if(checkDriverState(Vcc))
	{
		//Add new current peak, only if in a meaningful region
		if(std::abs(_currentPID.getLastOutput()) > 0.1)
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

}

bool
CurrentPID::onConfigure()
{
	if(!isConfigured()) {
		return false;
	}

    //Set PWM gain
	_sign = (uint8_t)configuration().invert ? -1 : 1;
	_Kpwm = 1.0 / configuration().maxV;
	_Ktorque = configuration().T/configuration().Kt;
	_controlCycles = configuration().controlCycles;

    
    //Compute PID tuning
    const float Kp = configuration().omegaC * configuration().L;
	const float Ti = configuration().L / configuration().R;
    const float Ts = _controlCycles/17.5e3; //TODO read from pwm driver

    //tune PID
	_currentPID.config(Kp, Ti, 0.0f, Ts, 0.0f, -configuration().maxV, configuration().maxV);
    
    //set pid setpoint
	_currentPID.set(0.0);
    
	if(!_pwm.configure()) {
		return false;
	}

	if(!_currentSensor.configure()) {
		return false;
	}

    return true;
}

bool
CurrentPID::onPrepareHW()
{
    // Start the ADC driver and conversion
	std::function<void(float,float)> adcCallback = std::bind(&CurrentPID::controlCallback, this, _1, _2);
	_currentSensor.setCallback(adcCallback);

	// Initialize the H bridge driver
	_pwm.init();
	_currentSensor.init();

    return true;
}

bool
CurrentPID::onPrepareMW()
{
    subscribe(_subscriber, configuration().topic);
    _subscriber.set_callback(CurrentPID::callback);

    return true;
}

bool
CurrentPID::callback(
   const actuator_msgs::Setpoint_f32& msg,
   core::mw::Node*                    node
)
{
   CurrentPID* _this = static_cast<CurrentPID*>(node);

   chSysLock();

   float currentSetpoint = _this->_sign*_this->_Ktorque*msg.value;

   _this->_currentPID.set(currentSetpoint);
   chSysUnlock();

   return true;
}

bool
CurrentPID::onStart() {
	_currentSensor.start();

	// Start the H bridge driver
	_pwm.start();

	float value = 0.0f;
	_pwm.set(value);

	return true;
}

bool
CurrentPID::onLoop()
{
	if (!this->spin(core::os::Time::ms(100)))
	{
		chSysLock();
		_currentPID.reset();
		chSysUnlock();
	}

    return true;
}

bool
CurrentPID::onStop() {
	// Stop the H bridge driver
	_pwm.stop();
	_currentSensor.stop();
	return true;
}

bool CurrentPID::checkDriverState(float Vcc)
{
    if(Vcc < configuration().Voff)
    {
    	_currentPID.reset();
    	_pwm.stop();
    	_underVoltage = true;
    }
    else if(_underVoltage && Vcc > configuration().Von)
    {
    	_pwm.start();
    	_underVoltage = false;
    }

    return !_underVoltage;
}

} /* namespace current_control */

}
