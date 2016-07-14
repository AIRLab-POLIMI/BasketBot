#include <mahony/MahonyFilter.hpp>

using namespace std;

#include <cmath>

static float invSqrtFull(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

namespace mahony {
void MahonyFilter::config(float Kp, float Ki, float Kacc, float Kmag,
		float deltaT) {
	_Kp = Kp;
	_Ki = Ki;
	_Kacc = Kacc;
	_Kmag = Kmag;
	_deltaT = deltaT;

	reset();
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

	if ((_measure.mag[0] == 0.0f) || (_measure.mag[1] == 0.0f)
			|| (_measure.mag[2] == 0.0f)) {
		valid_mag = false;
	}

	normalizeAccMeasure();

	if (valid_mag) {
		normalizeMagMeasure();
	}
	computeAttMatrix(attitude_matrix);

	referenceDirectionEarthAcc(attitude_matrix, v_acc_hat);

	if (valid_mag == 1) {
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

	attitude.linear_acceleration[0] = omega[0];
	attitude.linear_acceleration[1] = omega[1];
	attitude.linear_acceleration[2] = omega[2];

	attitude.angular_velocity[0] = omega[0];
	attitude.angular_velocity[1] = omega[1];
	attitude.angular_velocity[2] = omega[2];

}

void MahonyFilter::reset()
{
	bias_p = 0;
	bias_q = 0;
	bias_r = 0;
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
	v_acc_hat[0] = -attitude_matrix[0][2];
	v_acc_hat[1] = -attitude_matrix[1][2];
	v_acc_hat[2] = -attitude_matrix[2][2];
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
	omeMes[0] = -(_Kacc
			* (_measure.acc[1] * v_acc_hat[2] - _measure.acc[2] * v_acc_hat[1]))
			/ 2
			- (_Kmag
					* (_measure.mag[1] * v_mag_hat[2]
							- _measure.mag[2] * v_mag_hat[1])) / 2;
	omeMes[1] = +(_Kacc
			* (_measure.acc[0] * v_acc_hat[2] - _measure.acc[2] * v_acc_hat[0]))
			/ 2
			+ (_Kmag
					* (_measure.mag[0] * v_mag_hat[2]
							- _measure.mag[2] * v_mag_hat[0])) / 2;
	omeMes[2] = -(_Kacc
			* (_measure.acc[0] * v_acc_hat[1] - _measure.acc[1] * v_acc_hat[0]))
			/ 2
			- (_Kmag
					* (_measure.mag[0] * v_mag_hat[1]
							- _measure.mag[1] * v_mag_hat[0])) / 2;
}

/*
 * drift estimation , no magnetometer
 */
void MahonyFilter::driftEstimation(float omeMes[3], float v_acc_hat[3]) {
	omeMes[0] = -(_Kacc
			* (_measure.acc[1] * v_acc_hat[2] - _measure.acc[2] * v_acc_hat[1]))
			/ 2;
	omeMes[1] = +(_Kacc
			* (_measure.acc[0] * v_acc_hat[2] - _measure.acc[2] * v_acc_hat[0]))
			/ 2;
	omeMes[2] = -(_Kacc
			* (_measure.acc[0] * v_acc_hat[1] - _measure.acc[1] * v_acc_hat[0]))
			/ 2;
}

/*
 * bias estimation
 */
void MahonyFilter::biasEstimation(float omeMes[3]) {
	bias_p += _Ki * omeMes[0] * _deltaT;
	bias_q += _Ki * omeMes[1] * _deltaT;
	bias_r += _Ki * omeMes[2] * _deltaT;
}

/*
 * gyro_yroscope depolarization
 */
void MahonyFilter::gyroDepolar(float omega[3], float omeMes[3]) {
	omega[0] = _measure.gyr[0] - (bias_p + _Kp * omeMes[0]);
	omega[1] = _measure.gyr[1] - (bias_q + _Kp * omeMes[1]);
	omega[2] = _measure.gyr[2] - (bias_r + _Kp * omeMes[2]);
}

/*
 * Rate of change of onboard_attitude_quaternion_data from angular rate
 */
void MahonyFilter::rateChangeQuaternionAngRate(float qdot[3],
		float omega[3]) {
	auto q = attitude.orientation;
	qdot[0] = 0.5 * (q[1] * omega[2] - q[2] * omega[1] + q[3] * omega[0]);
	qdot[1] = 0.5 * (-q[0] * omega[2] + q[2] * omega[0] + q[3] * omega[1]);
	qdot[2] = 0.5 * (q[0] * omega[1] - q[1] * omega[0] + q[3] * omega[2]);
	qdot[3] = 0.5 * (-q[0] * omega[0] - q[1] * omega[1] - q[2] * omega[2]);
}

/*
 * Integrate rate of change of onboard_attitude_quaternion_data to yield onboard_attitude_quaternion_data
 */
void MahonyFilter::integrateRateChangeQuaternion(float qdot[3]) {
	auto q = attitude.orientation;
	q[0] += qdot[0] * _deltaT;
	q[1] += qdot[1] * _deltaT;
	q[2] += qdot[2] * _deltaT;
	q[3] += qdot[3] * _deltaT;
}

/*
 * onboard_attitude_quaternion_data normalization
 */
void MahonyFilter::normalizeQuaternion() {
	auto q = attitude.orientation;
	float recipNorm = invSqrtFull(
			q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

/*
 * atitude matrix computation
 */
void MahonyFilter::computeAttMatrix(float attitude_matrix[3][3]) {
	auto q = attitude.orientation;
	attitude_matrix[0][0] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2]
			+ q[3] * q[3];
	attitude_matrix[0][1] = 2 * q[0] * q[1] + 2 * q[2] * q[3];
	attitude_matrix[0][2] = 2 * q[0] * q[2] - 2 * q[1] * q[3];
	attitude_matrix[1][0] = 2 * q[0] * q[1] - 2 * q[2] * q[3];
	attitude_matrix[1][1] = -q[0] * q[0] + q[1] * q[1] - q[2] * q[2]
			+ q[3] * q[3];
	attitude_matrix[1][2] = 2 * q[0] * q[3] + 2 * q[1] * q[2];
	attitude_matrix[2][0] = 2 * q[0] * q[2] + 2 * q[1] * q[3];
	attitude_matrix[2][1] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
	attitude_matrix[2][2] = -q[0] * q[0] - q[1] * q[1] + q[2] * q[2]
			+ q[3] * q[3];
}
}
