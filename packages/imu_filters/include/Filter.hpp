#pragma once

#include <imu_filters/Measurement.hpp>
#include <Core/MW/Array.hpp>

namespace imu_filters
{

   float invSqrtFull(float x); //TODO move

   class Filter
   {
   public:
	   virtual void config(float deltaT) = 0;
	   virtual void operator()(const measurement& measure) = 0;
	   virtual void reset() = 0;

   protected:
		void computeAttMatrix(float attitude_matrix[3][3]);
		void computeQuaternion(float x[3], float y[3], float z[3]);
		void crossProduct(const float a[3], const float b[3], float c[3]);
		void normalizeQuaternion();
		void initPose();

   public:
		Core::MW::Array<float, 4> attitude;
		Core::MW::Array<float, 3> linear_acceleration;
		Core::MW::Array<float, 3> angular_velocity;
       
   protected:
        measurement _measure;
       
   };
   
}
