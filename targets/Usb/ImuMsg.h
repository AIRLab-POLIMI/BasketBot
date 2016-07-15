#ifndef IMUMSG_H_
#define IMUMSG_H_

#include <r2p/Message.hpp>

namespace r2p {

class Imu_f32: public Message {
public:
	float quaternion[4];
	float angular_velocity[3];
	float linear_acceleration[3];
} R2P_PACKED;


}

#endif
