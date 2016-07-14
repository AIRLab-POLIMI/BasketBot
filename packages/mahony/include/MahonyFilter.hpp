#pragma once

#include <mahony/Measurement.hpp>
#include <sensor_msgs/Imu_f32.hpp>

namespace mahony
{
   class MahonyFilter
   {
   public:
	   void config(float Kp, float Ki, float Kacc, float Kmag, float deltaT);
	   void operator()(const measurement& measure);
	   void reset();

   private:
		void normalizeAccMeasure();
		void normalizeMagMeasure();
		void referenceDirectionEarthAcc(float attitude_matrix[3][3], float v_acc_hat[3]);
		void referenceDirectionEarthMag(float attitude_matrix[3][3],float h[3],float v_mag_hat[3]);
		void driftEstimationMag(float omeMes[3],float v_mag_hat[3],float v_acc_hat[3]);
		void driftEstimation(float omeMes[3],float v_acc_hat[3]);
		void biasEstimation(float omeMes[3]);
		void gyroDepolar(float omega[3],float omeMes[3]);
		void rateChangeQuaternionAngRate(float qdot[3],float omega[3]);
		void integrateRateChangeQuaternion(float qdot[3]);
		void normalizeQuaternion();
		void computeAttMatrix(float attitude_matrix[3][3]);

   public:
		sensor_msgs::Imu_f32 attitude;

   private:
		float _Kp;
		float _Ki;
		float _Kacc;
		float _Kmag;
		float _deltaT;

		float bias_p, bias_q, bias_r;
		measurement _measure;

   };

}
