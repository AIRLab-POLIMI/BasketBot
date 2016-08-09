#include <core/current_control/Broadcaster.hpp>

namespace core {

namespace current_control {

Broadcaster::Broadcaster(const char* name, CurrentSensor& currentSensor,
		core::os::Thread::PriorityEnum priority) :
		      CoreNode::CoreNode(name, priority),
			  core::mw::CoreConfigurable<core::current_control::BroadcasterConfiguration>(name),
			  _currentSensor(currentSensor)
{
	_current = 0;
}

Broadcaster::~Broadcaster() {

}

bool Broadcaster::onConfigure() {

	_deltaT = core::os::Time::s(1.0/configuration().frequency);

	return true;

}

bool Broadcaster::onPrepareMW() {

	advertise(_publisher, configuration().topic);

	_stamp = core::os::Time::now();
	return true;
}

bool Broadcaster::onLoop() {
	core::os::Thread::sleep_until(_stamp+_deltaT);

	core::actuator_msgs::Setpoint_f32* msgp;

	if(_publisher.alloc(msgp))
	{
		float current;
		_currentSensor.update();
		_currentSensor.get(current);
		msgp->value = current;
		_publisher.publish(msgp);
	}

	_stamp = core::os::Time::now();

	return true;
}

}

}

