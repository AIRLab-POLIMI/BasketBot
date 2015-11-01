/*
 * ExtraMsgs.h
 *
 *  Created on: 01/nov/2015
 *      Author: dave
 */

#ifndef EXTRAMSGS_H_
#define EXTRAMSGS_H_

#include <r2p/Message.hpp>

namespace r2p {

class TwistMsg: public Message {
public:
	float angular[3];
	float linear[3];
}R2P_PACKED;

class CurrentMsg: public Message {
public:
	float value;
}R2P_PACKED;

class Current2Msg: public Message {
public:
	float value[2];
}R2P_PACKED;

class Current3Msg: public Message {
public:
	float value[3];
}R2P_PACKED;

}

#endif /* EXTRAMSGS_H_ */
