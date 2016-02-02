/*
 * magwdick_node.cpp
 *
 *  Created on: 02/feb/2016
 *      Author: dave
 */

#include "madgwick_node.hpp"

#include "l3gd20h.h"
#include "lsm303d.h"
#include "madgwick.h"

#include <r2p/Middleware.hpp>
#include <r2p/msg/imu.hpp>

static madgwick_node_conf madgwick_default_conf = { 10 };

/*
 * Madgwick node
 */
extern gyro_data_t gyro_data;
extern acc_data_t acc_data;
extern mag_data_t mag_data;

static const SPIConfig spi1cfg = { 0, 0, 0, SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA , 0};

static const EXTConfig extcfg = { {
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOB, l3gd20h_drdy_callback },
	{ EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA, lsm303_int1_cb },
	{ EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA, lsm303_int2_cb },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL }
} };

msg_t madgwick_node(void *arg) {

	madgwick_node_conf* conf;
	if(arg != NULL)
		conf = (madgwick_node_conf*) arg;
	else
		conf = &madgwick_default_conf;


	r2p::Node node("madgwick");
	r2p::Publisher<r2p::IMUMsg> imu_pub;
	r2p::Publisher<r2p::tIMURaw9> raw_pub;
	attitude_t attitude_data;
	systime_t time;

	(void) arg;
	chRegSetThreadName("madgwick");

	spiStart(&SPID1, &spi1cfg);
	extStart(&EXTD1, &extcfg);

	chThdSleepMilliseconds(500);

	gyroRun(&SPID1, NORMALPRIO);
	accRun(&SPID1, NORMALPRIO);
	magRun(&SPID1, NORMALPRIO);

	node.advertise(imu_pub, "imu");
	node.advertise(raw_pub, "imu_raw");

	time = chTimeNow();

	for (;;) {
		MadgwickAHRSupdateIMU((gyro_data.y / 57.143) * 3.141592 / 180.0, (-gyro_data.x / 57.143) * 3.141592 / 180.0,
				(gyro_data.z / 57.143) * 3.141592 / 180.0, acc_data.x / 1000.0, acc_data.y / 1000.0,
				acc_data.z / 1000.0);
		getMadAttitude(&attitude_data);

		r2p::Time stamp = r2p::Time::now();
		uint32_t sec = stamp.to_us_raw() / 1000000;
		uint32_t nsec = (stamp.to_us_raw() % 1000000)*1000;


		r2p::IMUMsg *msgp;
		if (imu_pub.alloc(msgp)) {
			msgp->roll = ((attitude_data.roll * 57.29578)); // rads to degrees
			msgp->pitch = ((attitude_data.pitch * 57.29578)); // rads to degrees
			msgp->yaw = ((attitude_data.yaw * 57.29578)); // rads to degrees

			imu_pub.publish(*msgp);
		}

		r2p::tIMURaw9 *rawp;
		if (raw_pub.alloc(rawp)) {
			rawp->acc_x = acc_data.x;
			rawp->acc_y = acc_data.y;
			rawp->acc_z = acc_data.z;
			rawp->gyro_x = gyro_data.y;
			rawp->gyro_y = -gyro_data.x;
			rawp->gyro_z = gyro_data.z;
			rawp->mag_x = mag_data.x;
			rawp->mag_y = mag_data.y;
			rawp->mag_z = mag_data.z;
			rawp->timestamp.nsec = nsec;
			rawp->timestamp.sec = sec;

			raw_pub.publish(*rawp);
		}

		time += MS2ST(conf->imuDt);
		chThdSleepUntil(time);
	}
	return CH_SUCCESS;
}



