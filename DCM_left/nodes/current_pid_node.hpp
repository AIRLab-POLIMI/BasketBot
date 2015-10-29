/*
 * CurrentLoop.h
 *
 *  Created on: 22/ott/2015
 *      Author: dave
 */

#ifndef DCM_MODULE_CURRENTLOOP_H_
#define DCM_MODULE_CURRENTLOOP_H_

#include <r2p/Message.hpp>

namespace r2p
{

class Current2Msg: public Message {
public:
	float value[2];
}R2P_PACKED;

struct current_pid_node_conf {
	const char * name;
	int index;
	float L;
	float R;
	float omegaC;
	float maxV;
};

msg_t current_pid2_node(void * arg);

}



#endif /* DCM_MODULE_CURRENTLOOP_H_ */
