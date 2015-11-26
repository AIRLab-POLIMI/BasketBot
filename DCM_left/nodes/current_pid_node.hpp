/*
 * CurrentLoop.h
 *
 *  Created on: 22/ott/2015
 *      Author: dave
 */

#ifndef DCM_MODULE_CURRENTLOOP_H_
#define DCM_MODULE_CURRENTLOOP_H_

#include "ExtraMsgs.h"

namespace r2p
{

struct current_pid_node_conf {
	const char * name;
	int index;
	float R;
	float L;
	float omegaC;
	float maxV;
};

struct current_publisher_node_conf {
	const char * name;
	const char * topic;
};

msg_t current_pid2_node(void * arg);
msg_t current_publisher_node(void * arg);

}



#endif /* DCM_MODULE_CURRENTLOOP_H_ */
