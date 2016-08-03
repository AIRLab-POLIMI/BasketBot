#pragma once

#include <core/imu_filters/Measurement.hpp>
#include <core/Array.hpp>

namespace core
{
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
		core::Array<float, 4> attitude;
		core::Array<float, 3> linear_acceleration;
		core::Array<float, 3> angular_velocity;
       
   protected:
        measurement _measure;
       
   };
   
}
}
