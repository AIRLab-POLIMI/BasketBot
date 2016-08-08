#pragma once

namespace quaternions
{

class Utils
{
public:
	static double getRoll(float q[4]);	
	static double getPitch(float q[4]);	
	static double getYaw(float q[4]);	

};



}
