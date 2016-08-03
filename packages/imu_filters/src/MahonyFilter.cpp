#include <core/imu_filters/MahonyFilter.hpp>

using namespace std;

#include <cmath>

namespace core
{
namespace imu_filters {
void MahonyFilter::config(float deltaT) {
	_Kp = configuration.Kp;
	_Ki = configuration.Ki;
	_Kacc = configuration.Kacc;
	_Kmag = configuration.Kmag;
	_deltaT = deltaT;

	bias_p = 0;
	bias_q = 0;
	bias_r = 0;

	needReset = true;
}

void MahonyFilter::operator()(const measurement& measure) {
	_measure = measure;

	float attitude_matrix[3][3];
	float v_acc_hat[3];
	float v_mag_hat[3];
	float h[3];
	float omeMes[3];
	float omega[3];
	float qdot[4];

	bool valid_mag = true;

	if ((_measure.mag[0] == 0.0f) && (_measure.mag[1] == 0.0f)
			&& (_measure.mag[2] == 0.0f)) {
		valid_mag = false;
	}

	normalizeAccMeasure();

	if (valid_mag) {
		normalizeMagMeasure();
	}

	if (needReset) {
		if (valid_mag)
			reset();
		return;
	}

	computeAttMatrix(attitude_matrix);

	referenceDirectionEarthAcc(attitude_matrix, v_acc_hat);

	if (valid_mag) {
		referenceDirectionEarthMag(attitude_matrix, h, v_mag_hat);
		driftEstimationMag(omeMes, v_mag_hat, v_acc_hat);
	} else {
		driftEstimation(omeMes, v_acc_hat);
	}

	biasEstimation(omeMes);

	gyroDepolar(omega, omeMes);

	rateChangeQuaternionAngRate(qdot, omega);

	integrateRateChangeQuaternion(qdot);

	normalizeQuaternion();

	linear_acceleration[0] = -measure.acc[0]*9.81;
	linear_acceleration[1] = -measure.acc[1]*9.81;
	linear_acceleration[2] = -measure.acc[2]*9.81;

	angular_velocity[0] = omega[0];
	angular_velocity[1] = omega[1];
	angular_velocity[2] = omega[2];

}

void MahonyFilter::reset() {
	bias_p = 0;
	bias_q = 0;
	bias_r = 0;

	initPose();

	needReset = false;
}

/*
 * normalize accelerometer measure
 */
void MahonyFilter::normalizeAccMeasure() {
	float recipNorm = invSqrtFull(
			pow(_measure.acc[0], 2) + pow(_measure.acc[1], 2)
					+ pow(_measure.acc[2], 2));
	_measure.acc[0] *= recipNorm;
	_measure.acc[1] *= recipNorm;
	_measure.acc[2] *= recipNorm;
}
/*
 * normalize magnetometer measure
 */
void MahonyFilter::normalizeMagMeasure() {
	float recipNorm = invSqrtFull(
			pow(_measure.mag[0], 2) + pow(_measure.mag[1], 2)
					+ pow(_measure.mag[2], 2));
	_measure.mag[0] *= recipNorm;
	_measure.mag[1] *= recipNorm;
	_measure.mag[2] *= recipNorm;
}

/*
 * reference direction accelerometer
 */
void MahonyFilter::referenceDirectionEarthAcc(float attitude_matrix[3][3],
		float v_acc_hat[3]) {
	v_acc_hat[0] = attitude_matrix[0][2];
	v_acc_hat[1] = attitude_matrix[1][2];
	v_acc_hat[2] = attitude_matrix[2][2];
}

/*
 * reference direction magnetomer
 */
void MahonyFilter::referenceDirectionEarthMag(float attitude_matrix[3][3],
		float h[3], float v_mag_hat[3]) {
	h[0] = _measure.mag[0] * attitude_matrix[0][0]
			+ _measure.mag[1] * attitude_matrix[1][0]
			+ _measure.mag[2] * attitude_matrix[2][0];
	h[1] = _measure.mag[0] * attitude_matrix[0][1]
			+ _measure.mag[1] * attitude_matrix[1][1]
			+ _measure.mag[2] * attitude_matrix[2][1];
	h[2] = _measure.mag[0] * attitude_matrix[0][2]
			+ _measure.mag[1] * attitude_matrix[1][2]
			+ _measure.mag[2] * attitude_matrix[2][2];

	float norm_h = sqrt(h[0] * h[0] + h[1] * h[1]);

	v_mag_hat[0] = attitude_matrix[0][2] * h[2]
			+ attitude_matrix[0][0] * norm_h;
	v_mag_hat[1] = attitude_matrix[1][2] * h[2]
			+ attitude_matrix[1][0] * norm_h;
	v_mag_hat[2] = attitude_matrix[2][2] * h[2]
			+ attitude_matrix[2][0] * norm_h;
}

/*
 * drift estimation with magnetometer
 */
void MahonyFilter::driftEstimationMag(float omeMes[3], float v_mag_hat[3],
		float v_acc_hat[3]) {
	omeMes[0] = +(_Kacc	* (_measure.acc[1] * v_acc_hat[2] - _measure.acc[2] * v_acc_hat[1])) / 2
			    +(_Kmag	* (_measure.mag[1] * v_mag_hat[2] - _measure.mag[2] * v_mag_hat[1])) / 2;
	omeMes[1] = -(_Kacc	* (_measure.acc[0] * v_acc_hat[2] - _measure.acc[2] * v_acc_hat[0])) / 2
			    -(_Kmag	* (_measure.mag[0] * v_mag_hat[2] - _measure.mag[2] * v_mag_hat[0])) / 2;
	omeMes[2] = +(_Kacc	* (_measure.acc[0] * v_acc_hat[1] - _measure.acc[1] * v_acc_hat[0])) / 2
			    +(_Kmag * (_measure.mag[0] * v_mag_hat[1] - _measure.mag[1] * v_mag_hat[0])) / 2;
}

/*
 * drift estimation, no magnetometer
 */
void MahonyFilter::driftEstimation(float omeMes[3], float v_acc_hat[3]) {
	omeMes[0] = +(_Kacc	* (_measure.acc[1] * v_acc_hat[2] - _measure.acc[2] * v_acc_hat[1])) / 2;
	omeMes[1] = -(_Kacc	* (_measure.acc[0] * v_acc_hat[2] - _measure.acc[2] * v_acc_hat[0])) / 2;
	omeMes[2] = +(_Kacc	* (_measure.acc[0] * v_acc_hat[1] - _measure.acc[1] * v_acc_hat[0])) / 2;
}

/*
 * bias estimation
 */
void MahonyFilter::biasEstimation(float omeMes[3]) {
	bias_p -= _Ki * omeMes[0] * _deltaT;
	bias_q -= _Ki * omeMes[1] * _deltaT;
	bias_r -= _Ki * omeMes[2] * _deltaT;
}

/*
 * gyro_yroscope depolarization
 */
void MahonyFilter::gyroDepolar(float omega[3], float omeMes[3]) {
	omega[0] = _measure.gyr[0] - bias_p + _Kp * omeMes[0];
	omega[1] = _measure.gyr[1] - bias_q + _Kp * omeMes[1];
	omega[2] = _measure.gyr[2] - bias_r + _Kp * omeMes[2];
}

/*
 * Rate of change of onboard_attitude_quaternion_data from angular rate
 */
void MahonyFilter::rateChangeQuaternionAngRate(float qdot[4], float omega[3]) {
	auto& q = attitude;
	qdot[0] = 0.5 * (q[1] * omega[2] - q[2] * omega[1] + q[3] * omega[0]);
	qdot[1] = 0.5 * (-q[0] * omega[2] + q[2] * omega[0] + q[3] * omega[1]);
	qdot[2] = 0.5 * (q[0] * omega[1] - q[1] * omega[0] + q[3] * omega[2]);
	qdot[3] = 0.5 * (-q[0] * omega[0] - q[1] * omega[1] - q[2] * omega[2]);
}

/*
 * Integrate rate of change of onboard_attitude_quaternion_data to yield onboard_attitude_quaternion_data
 */
void MahonyFilter::integrateRateChangeQuaternion(float qdot[4]) {
	attitude[0] += qdot[0] * _deltaT;
	attitude[1] += qdot[1] * _deltaT;
	attitude[2] += qdot[2] * _deltaT;
	attitude[3] += qdot[3] * _deltaT;
}

}

}
