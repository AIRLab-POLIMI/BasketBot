/*
 * handlers.cpp
 *
 *  Created on: 10/nov/2015
 *      Author: dave
 */

#include "urosHandlers.h"

#include "hal.h"

#include <r2p/common.hpp>
#include <r2p/Middleware.hpp>
#include <r2p/Node.hpp>
#include <r2p/Topic.hpp>
#include <r2p/Publisher.hpp>
#include <r2p/Subscriber.hpp>

#include "ExtraMsgs.h"

extern r2p::Publisher<r2p::Current2Msg> vel_pub;
/*void sub_cb__tiltone__velocity(struct msg__geometry_msgs__Twist *msg) {
 palClearPad(LED1_GPIO, LED1);


 }*/

void sub_cb__tiltone__setpoint(struct msg__std_msgs__Float32 *msg) {
	r2p::Current2Msg* msgp;

	if (vel_pub.alloc(msgp)) {
		msgp->value[0] = msg->data;
		msgp->value[1] = msg->data;

		palClearPad(LED1_GPIO, LED1);
	}

	vel_pub.publish(*msgp);
}

