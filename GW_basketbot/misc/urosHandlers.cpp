/**
 * @file    urosHandlers.c
 * @author  TODO
 *
 * @brief   TCPROS topic and service handlers.
 */

/*===========================================================================*/
/* HEADER FILES                                                              */
/*===========================================================================*/

#include "urosHandlers.h"

#include <urosNode.h>
#include <urosTcpRos.h>
#include <urosUser.h>

#include <r2p/common.hpp>
#include <r2p/Middleware.hpp>
#include <r2p/Node.hpp>
#include <r2p/Publisher.hpp>
#include <r2p/Subscriber.hpp>

#include "ExtraMsgs.h"

/*===========================================================================*/
/* GLOBAL VARIABLES                                                          */
/*===========================================================================*/
r2p::Node current_node("ucurrentpub", false);
r2p::Subscriber<r2p::CurrentMsg, 5> current_sub;

r2p::Node vel_node("uvelpub", false);
r2p::Publisher<r2p::TwistMsg> vel_pub;

extern int activity;

/*===========================================================================*/
/* PUBLISHED TOPIC FUNCTIONS                                                 */
/*===========================================================================*/

/** @addtogroup tcpros_pubtopic_funcs */
/** @{ */

/*~~~ PUBLISHED TOPIC: /tiltone/current ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Topic <tt>/tiltone/current</tt> publisher */
/** @{ */

/**
 * @brief   TCPROS <tt>/tiltone/current</tt> published topic handler.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @return
 *          Error code.
 */
uros_err_t pub_tpc__tiltone__current(UrosTcpRosStatus *tcpstp) {

	r2p::CurrentMsg *msgp;
	static bool first_time = true;

	if (first_time) {
		current_node.subscribe(current_sub, "tilt");
		first_time = false;
	}

	current_node.set_enabled(true);

	/* Message allocation and initialization.*/
	UROS_TPC_INIT_S(msg__std_msgs__Float32);

	/* Published messages loop.*/
	while (!urosTcpRosStatusCheckExit(tcpstp)) {
		if (current_node.spin(r2p::Time::ms(1))) { //TODO FIX TIME
			while (current_sub.fetch(msgp)) {
				msg.data = msgp->value;
				current_sub.release(*msgp);

				/* Send the message.*/
				UROS_MSG_SEND_LENGTH(&msg, msg__std_msgs__Float32);
				UROS_MSG_SEND_BODY(&msg, msg__std_msgs__Float32);

				/* Dispose the contents of the message.*/
				clean_msg__std_msgs__Float32(&msg);
			}
		}

	}
	tcpstp->err = UROS_OK;

	_finally:
	/* Message deinitialization and deallocation.*/
	UROS_TPC_UNINIT_S(msg__std_msgs__Float32);
	return tcpstp->err;
}

/** @} */

/** @} */

/*===========================================================================*/
/* SUBSCRIBED TOPIC FUNCTIONS                                                */
/*===========================================================================*/

/** @addtogroup tcpros_subtopic_funcs */
/** @{ */

/*~~~ SUBSCRIBED TOPIC: /tiltone/velocity ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Topic <tt>/tiltone/velocity</tt> subscriber */
/** @{ */

/**
 * @brief   TCPROS <tt>/tiltone/velocity</tt> subscribed topic handler.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @return
 *          Error code.
 */
uros_err_t sub_tpc__tiltone__velocity(UrosTcpRosStatus *tcpstp) {

	r2p::TwistMsg *msgp;
	static bool first_time = true;

	if (first_time) {
		vel_node.advertise(vel_pub, "velocity", r2p::Time::INFINITE);
		first_time = false;
	}

	vel_node.set_enabled(true);

	/* Message allocation and initialization.*/
	UROS_TPC_INIT_S(msg__geometry_msgs__Twist);

	/* Subscribed messages loop.*/
	while (!urosTcpRosStatusCheckExit(tcpstp)) {
		/* Receive the next message.*/
		UROS_MSG_RECV_LENGTH()
		;
		UROS_MSG_RECV_BODY(&msg, msg__geometry_msgs__Twist);

		if (vel_pub.alloc(msgp)) {
			msgp->angular[0] = msg.angular.x;
			msgp->angular[1] = msg.angular.y;
			msgp->angular[2] = msg.angular.z;

			msgp->linear[0] = msg.linear.x;
			msgp->linear[1] = msg.linear.y;
			msgp->linear[2] = msg.linear.z;
		}

		vel_pub.publish(*msgp);

		activity = 1;

		/* Dispose the contents of the message.*/
		clean_msg__geometry_msgs__Twist(&msg);
	}
	tcpstp->err = UROS_OK;

	_finally:
	/* Message deinitialization and deallocation.*/
	UROS_TPC_UNINIT_S(msg__geometry_msgs__Twist);
	return tcpstp->err;
}

/** @} */

/** @} */

/*===========================================================================*/
/* PUBLISHED SERVICE FUNCTIONS                                               */
/*===========================================================================*/

/** @addtogroup tcpros_pubservice_funcs */
/** @{ */

/* There are no published services.*/

/** @} */

/*===========================================================================*/
/* CALLED SERVICE FUNCTIONS                                                  */
/*===========================================================================*/

/** @addtogroup tcpros_callservice_funcs */
/** @{ */

/* There are no called services.*/

/** @} */

/*===========================================================================*/
/* GLOBAL FUNCTIONS                                                          */
/*===========================================================================*/

/** @addtogroup tcpros_funcs */
/** @{ */

/**
 * @brief   Registers all the published topics to the Master node.
 * @note    Should be called at node initialization.
 */
void urosHandlersPublishTopics(void) {

	/* /tiltone/current */
	urosNodePublishTopicSZ("/tiltone/current", "std_msgs/Float32",
			(uros_proc_f) pub_tpc__tiltone__current, uros_nulltopicflags);
}

/**
 * @brief   Unregisters all the published topics to the Master node.
 * @note    Should be called at node shutdown.
 */
void urosHandlersUnpublishTopics(void) {

	/* /tiltone/current */
	urosNodeUnpublishTopicSZ("/tiltone/current");
}

/**
 * @brief   Registers all the subscribed topics to the Master node.
 * @note    Should be called at node initialization.
 */
void urosHandlersSubscribeTopics(void) {

	/* /tiltone/velocity */
	urosNodeSubscribeTopicSZ("/tiltone/velocity", "geometry_msgs/Twist",
			(uros_proc_f) sub_tpc__tiltone__velocity, uros_nulltopicflags);
}

/**
 * @brief   Unregisters all the subscribed topics to the Master node.
 * @note    Should be called at node shutdown.
 */
void urosHandlersUnsubscribeTopics(void) {

	/* /tiltone/velocity */
	urosNodeUnsubscribeTopicSZ("/tiltone/velocity");
}

/**
 * @brief   Registers all the published services to the Master node.
 * @note    Should be called at node initialization.
 */
void urosHandlersPublishServices(void) {

	/* No services to publish.*/
}

/**
 * @brief   Unregisters all the published services to the Master node.
 * @note    Should be called at node shutdown.
 */
void urosHandlersUnpublishServices(void) {

	/* No services to unpublish.*/
}

/** @} */
