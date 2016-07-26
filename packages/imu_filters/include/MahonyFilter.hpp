#pragma once

#include <imu_filters/Filter.hpp>
#include <imu_filters/MahonyConfiguration.hpp>

namespace imu_filters
{
   class MahonyFilter : public Filter
   {
   public:
	   virtual void config(float deltaT);
	   virtual void operator()(const measurement& measure);
	   virtual void reset();

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

   public:
         MahonyConfiguration configuration;

   private:
		float _Kp;
		float _Ki;
		float _Kacc;
		float _Kmag;
		float _deltaT;

		float bias_p, bias_q, bias_r;

		bool needReset;

   };

}
