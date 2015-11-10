/*
 * broadcaster_node.hpp
 *
 *  Created on: 10/nov/2015
 *      Author: dave
 */

#ifndef NODES_BROADCASTER_NODE_HPP_
#define NODES_BROADCASTER_NODE_HPP_

#include "ExtraMsgs.h"

namespace r2p {

struct broadcaster_node_conf {
	const char * name;
	const char * topicIn;
	const char * topicOut;
};

msg_t broadcaster_node(void * arg);

}

#endif /* NODES_BROADCASTER_NODE_HPP_ */
