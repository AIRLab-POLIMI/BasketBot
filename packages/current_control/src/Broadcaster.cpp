/*
 * broadcaster_node.cpp
 *
 *  Created on: 10/nov/2015
 *      Author: dave
 */

/*#include "ch.h"
#include "hal.h"

#include "r2p/Middleware.hpp"

#include "broadcaster_node.hpp"

namespace r2p {*/

/*===========================================================================*/
/* Bufferizing node.                                                         */
/*===========================================================================*/

/*static broadcaster_node_conf defaultConf = { "broadcaster_node", "bits",
		"bits_packed", 50 };

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

	for (;;) {

		if (node.spin()) {

			// fetch data
			while (calibration_sub.fetch(msgp_in) && count <  conf->factor) {
				count++;
				float value = msgp_in->value;
				calibration_sub.release(*msgp_in);

				//publish data at less rate
				if (count == conf->factor) {
					if (calibration_pub.alloc(msgp_out)) {
						msgp_out->value = value;
						calibration_pub.publish(*msgp_out);
					}

					count = 0;
				}
			}
		}

	}

	return CH_SUCCESS;
}

}*/

