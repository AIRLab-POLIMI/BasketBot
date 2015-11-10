/*
 * broadcaster_node.cpp
 *
 *  Created on: 10/nov/2015
 *      Author: dave
 */

#include "ch.h"
#include "hal.h"

#include "r2p/Middleware.hpp"

#include "broadcaster_node.hpp"

namespace r2p {

/*===========================================================================*/
/* Bufferizing node.                                                         */
/*===========================================================================*/

static broadcaster_node_conf defaultConf = { "calibration_node", "bits",
		"bits_packed" };

msg_t broadcaster_node(void* arg) {
	broadcaster_node_conf* conf;
	if (arg != NULL)
		conf = (broadcaster_node_conf *) arg;
	else
		conf = &defaultConf;

	Node node(conf->name);

	Subscriber<FloatMsg, 5> calibration_sub;
	FloatMsg * msgp_in;

	Publisher<FloatMsg> calibration_pub;
	FloatMsg * msgp_out;

	chRegSetThreadName(conf->name);

	node.subscribe(calibration_sub, conf->topicIn);
	node.advertise(calibration_pub, conf->topicOut);

	int count = 0;
	float buffer[20];

	for (;;) {

		if (node.spin(r2p::Time::ms(1000))) {

			// fetch data
			if (calibration_sub.fetch(msgp_in)) {
				buffer[count++] = msgp_in->value;
				calibration_sub.release(*msgp_in);
			}

			// publish mean data
			if (count == 20) {
				if (calibration_pub.alloc(msgp_out)) {

					float value = 0;
					for (int i = 0; i < 20; i++) {
						value += buffer[i];
					}

					msgp_out->value = static_cast<float>(value) / 20.0;

					calibration_pub.publish(*msgp_out);
				}

				count = 0;
			}
		}

	}

	return CH_SUCCESS;
}

}

