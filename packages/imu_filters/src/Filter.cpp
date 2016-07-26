#include <imu_filters/Filter.hpp>

#include <cmath>

namespace imu_filters
{

float invSqrtFull(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void Filter::computeAttMatrix(float attitude_matrix[3][3]) {
	auto& q = attitude;
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

void Filter::computeQuaternion(float x[3], float y[3], float z[3]) {
	double w = sqrt(1.0 + x[0] + y[1] + z[2]) / 2.0;
	double w4 = (4.0 * w);
	attitude[0] = (z[1] - y[2]) / w4;
	attitude[1] = (x[2] - z[0]) / w4;
	attitude[2] = (y[0] - x[1]) / w4;
	attitude[3] = w;
}

void Filter::normalizeQuaternion() {
	auto& q = attitude;
	float recipNorm = invSqrtFull(
			q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

void Filter::crossProduct(const float a[3], const float b[3], float c[3])
{
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
}

void Filter::initPose() {
	float x[3];
	float y[3];
	float* z = _measure.acc;

	//compute cross product to find y axis
	crossProduct(z, _measure.mag, y);

	//compute cross product to find x axis
	crossProduct(y, z, x);

	//compute quaternion from attitude
	computeQuaternion(x, y, z);
}

}
