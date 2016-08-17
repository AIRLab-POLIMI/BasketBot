#include <core/quaternions/Utils.hpp>

#include <cmath>

namespace quaternions
{

double Utils::getRoll(float q[4])
{
	return atan2(q[1]*q[2]+q[3]*q[0], 0.5 - (q[0]*q[0]+q[1]*q[1]));
}
	
double Utils::getPitch(float q[4])
{
	return asin(q[3]*q[1]-q[0]*q[2]);
}
	
double Utils::getYaw(float q[4])
{
	return atan2(q[0]*q[1]+q[3]*q[2], 0.5 - (q[1]*q[1]+q[2]*q[2]));
}


}
