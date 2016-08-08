#include <core/quaternions/Utils.hpp>

#include <cmath>

namespace quaternions
{

double Utils::getRoll(float q[4])
{
	return atan2(2.0*(q[1]*q[2]+q[0]*q[3]),(0.5*(q[2]*q[2]+q[3]*q[3])));
}
	
double Utils::getPitch(float q[4])
{
	return asin(2.0*(q[0]*q[2] - q[3]*q[1]));
}
	
double Utils::getYaw(float q[4])
{
	return atan2((q[2]*q[3]+q[0]*q[1]),(0.5-(q[1]*q[1]+q[2]*q[2])));
}


}
