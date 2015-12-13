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

#include <r2p/msg/imu.hpp>

/*===========================================================================*/
/* GLOBAL VARIABLES                                                          */
/*===========================================================================*/
r2p::Node imuraw_node("uimurawsub", false);
r2p::Subscriber<r2p::IMURaw9, 5> imuraw_sub;

/*===========================================================================*/
/* PUBLISHED TOPIC FUNCTIONS                                                 */
/*===========================================================================*/

/** @addtogroup tcpros_pubtopic_funcs */
/** @{ */

/*~~~ PUBLISHED TOPIC: /tiltone/imu_raw ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Topic <tt>/tiltone/imu_raw</tt> publisher */
/** @{ */

/**
 * @brief   TCPROS <tt>/tiltone/imu_raw</tt> published topic handler.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @return
 *          Error code.
 */
uros_err_t pub_tpc__tiltone__imu_raw(UrosTcpRosStatus *tcpstp) {

	r2p::IMURaw9 *msgp;
	static bool first_time = true;

	if (first_time) {
		imuraw_node.subscribe(imuraw_sub, "imu_raw");
		first_time = false;
	}

	imuraw_node.set_enabled(true);

	/* Message allocation and initialization.*/
	UROS_TPC_INIT_S(msg__hardware_tools_msgs__ImuRaw);

	/* Published messages loop.*/
	while (!urosTcpRosStatusCheckExit(tcpstp)) {
		if (imuraw_node.spin(r2p::Time::ms(1))) { //TODO FIX TIME
			while (imuraw_sub.fetch(msgp)) {
				//accelerometer
				msg.data[0] = msgp->acc_x;
				msg.data[1] = msgp->acc_y;
				msg.data[2] = msgp->acc_z;

				//gyroscope
				msg.data[3] = msgp->gyro_x;
				msg.data[4] = msgp->gyro_y;
				msg.data[5] = msgp->gyro_z;

				//magnetormeter
				msg.data[6] = msgp->mag_x;
				msg.data[7] = msgp->mag_y;
				msg.data[8] = msgp->mag_z;

				//TODO timestamp

				imuraw_sub.release(*msgp);

				/* Send the message.*/
				UROS_MSG_SEND_LENGTH(&msg, msg__hardware_tools_msgs__ImuRaw);
				UROS_MSG_SEND_BODY(&msg, msg__hardware_tools_msgs__ImuRaw);

				/* Dispose the contents of the message.*/
				clean_msg__hardware_tools_msgs__ImuRaw(&msg);
			}
		}
	}
	tcpstp->err = UROS_OK;

	_finally:
	/* Fetch pending messages and disable r2p node. */
	imuraw_node.set_enabled(false);
	while (imuraw_sub.fetch(msgp)) {
		imuraw_sub.release(*msgp);
	}
	/* Message deinitialization and deallocation.*/
	UROS_TPC_UNINIT_S(msg__hardware_tools_msgs__ImuRaw);
	return tcpstp->err;
}

/** @} */

/** @} */

/*===========================================================================*/
/* SUBSCRIBED TOPIC FUNCTIONS                                                */
/*===========================================================================*/

/** @addtogroup tcpros_subtopic_funcs */
/** @{ */

/*~~~ SUBSCRIBED TOPIC: /tiltone/setpoint ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Topic <tt>/tiltone/setpoint</tt> subscriber */
/** @{ */

/**
 * @brief   TCPROS <tt>/tiltone/setpoint</tt> subscribed topic handler.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @return
 *          Error code.
 */
uros_err_t sub_tpc__tiltone__setpoint(UrosTcpRosStatus *tcpstp) {

	/* Message allocation and initialization.*/
	UROS_TPC_INIT_S(msg__std_msgs__Float32);

	/* Subscribed messages loop.*/
	while (!urosTcpRosStatusCheckExit(tcpstp)) {
		/* Receive the next message.*/
		UROS_MSG_RECV_LENGTH()
		;
		UROS_MSG_RECV_BODY(&msg, msg__std_msgs__Float32);

		/* TODO: Process the received message.*/

		/* Dispose the contents of the message.*/
		clean_msg__std_msgs__Float32(&msg);
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

	/* /tiltone/imu_raw */
	urosNodePublishTopicSZ("/tiltone/imu_raw", "hardware_tools_msgs/ImuRaw",
			(uros_proc_f) pub_tpc__tiltone__imu_raw, uros_nulltopicflags);
}

/**
 * @brief   Unregisters all the published topics to the Master node.
 * @note    Should be called at node shutdown.
 */
void urosHandlersUnpublishTopics(void) {

	/* /tiltone/imu_raw */
	urosNodeUnpublishTopicSZ("/tiltone/imu_raw");
}

/**
 * @brief   Registers all the subscribed topics to the Master node.
 * @note    Should be called at node initialization.
 */
void urosHandlersSubscribeTopics(void) {

	/* /tiltone/setpoint */
	urosNodeSubscribeTopicSZ("/tiltone/setpoint", "std_msgs/Float32",
			(uros_proc_f) sub_tpc__tiltone__setpoint, uros_nulltopicflags);
}

/**
 * @brief   Unregisters all the subscribed topics to the Master node.
 * @note    Should be called at node shutdown.
 */
void urosHandlersUnsubscribeTopics(void) {

	/* /tiltone/setpoint */
	urosNodeUnsubscribeTopicSZ("/tiltone/setpoint");
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

