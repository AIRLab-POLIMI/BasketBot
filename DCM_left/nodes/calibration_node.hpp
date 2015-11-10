/*
 * calibration_node.hpp
 *
 *  Created on: 02/nov/2015
 *      Author: dave
 */

#ifndef NODES_CALIBRATION_NODE_HPP_
#define NODES_CALIBRATION_NODE_HPP_

#include "ExtraMsgs.h"

namespace r2p {

struct calibration_pub_node_conf {
	const char * name;
	const char * topic;
};

msg_t motor_calibration_node(void * arg);

}

#endif /* NODES_CALIBRATION_NODE_HPP_ */
