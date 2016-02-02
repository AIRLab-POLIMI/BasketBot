/*
 * madgwick_node.hpp
 *
 *  Created on: 02/feb/2016
 *      Author: dave
 */

#ifndef NODES_MADGWICK_NODE_HPP_
#define NODES_MADGWICK_NODE_HPP_

#include "ch.h"

struct madgwick_node_conf {
	unsigned int imuDt;
};

msg_t madgwick_node(void *arg);



#endif /* NODES_MADGWICK_NODE_HPP_ */
