/*
 * handlers.cpp
 *
 *  Created on: 10/nov/2015
 *      Author: dave
 */

#include "urosHandlers.h"

#include <r2p/common.hpp>
#include <r2p/Middleware.hpp>
#include <r2p/Node.hpp>
#include <r2p/Topic.hpp>
#include <r2p/Publisher.hpp>
#include <r2p/Subscriber.hpp>

#include "ExtraMsgs.h"

extern r2p::Publisher<r2p::Current2Msg> vel_pub;

void sub_cb__tiltone__velocity(msg__geometry_msgs__Twist *msg) {
	r2p::Current2Msg* msgp;

	if (vel_pub.alloc(msgp)) {
		msgp->value[0] = msg->linear.x;
		msgp->value[1] = msg->linear.x;
	}

	vel_pub.publish(*msgp);

}

