/**
 * @file    urosMsgTypes.c
 * @author  TODO
 *
 * @brief   TCPROS message and service descriptor functions.
 */

/*===========================================================================*/
/* HEADER FILES                                                              */
/*===========================================================================*/

#include "urosMsgTypes.h"

/*===========================================================================*/
/* MESSAGE CONSTANTS                                                         */
/*===========================================================================*/

/** @addtogroup tcpros_msg_consts */
/** @{ */

/* There are no message constants.*/

/** @} */

/*===========================================================================*/
/* SERVICE CONSTANTS                                                         */
/*===========================================================================*/

/** @addtogroup tcpros_srv_consts */
/** @{ */

/* There are no service constants.*/

/** @} */

/*===========================================================================*/
/* MESSAGE FUNCTIONS                                                         */
/*===========================================================================*/

/** @addtogroup tcpros_msg_funcs */
/** @{ */

/*~~~ MESSAGE: std_msgs/Header ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Message <tt>std_msgs/Header</tt> */
/** @{ */

/**
 * @brief   Content length of a TCPROS <tt>std_msgs/Header</tt> message.
 *
 * @param[in,out] objp
 *          Pointer to an initialized <code>struct msg__std_msgs__Header</code> object.
 * @return
 *          Length of the TCPROS message contents, in bytes.
 */
size_t length_msg__std_msgs__Header(
  struct msg__std_msgs__Header *objp
) {
  size_t length = 0;

  urosAssert(objp != NULL);

  length += sizeof(uint32_t);
  length += sizeof(uros_time_t);
  length += sizeof(uint32_t) + objp->frame_id.length;

  return length;
}

/**
 * @brief   Initializes a TCPROS <tt>std_msgs/Header</tt> message.
 *
 * @param[in,out] objp
 *          Pointer to an allocated <code>struct msg__std_msgs__Header</code> object.
 * @return
 *          Error code.
 */
void init_msg__std_msgs__Header(
  struct msg__std_msgs__Header *objp
) {
  urosAssert(objp != NULL);

  urosStringObjectInit(&objp->frame_id);
}

/**
 * @brief   Cleans a TCPROS <tt>std_msgs/Header</tt> message.
 *
 * @param[in,out] objp
 *          Pointer to an initialized <code>struct msg__std_msgs__Header</code> object, or @p NULL.
 * @return
 *          Error code.
 */
void clean_msg__std_msgs__Header(
  struct msg__std_msgs__Header *objp
) {
  if (objp == NULL) { return; }

  urosStringClean(&objp->frame_id);
}

/**
 * @brief   Receives a TCPROS <tt>std_msgs/Header</tt> message.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @param[out] objp
 *          Pointer to an initialized <code>struct msg__std_msgs__Header</code> object.
 * @return
 *          Error code.
 */
uros_err_t recv_msg__std_msgs__Header(
  UrosTcpRosStatus *tcpstp,
  struct msg__std_msgs__Header *objp
) {
  urosAssert(tcpstp != NULL);
  urosAssert(urosConnIsValid(tcpstp->csp));
  urosAssert(objp != NULL);
#define _CHKOK { if (tcpstp->err != UROS_OK) { goto _error; } }

  urosTcpRosRecvRaw(tcpstp, objp->seq); _CHKOK
  urosTcpRosRecvRaw(tcpstp, objp->stamp); _CHKOK
  urosTcpRosRecvString(tcpstp, &objp->frame_id); _CHKOK

  return tcpstp->err = UROS_OK;
_error:
  clean_msg__std_msgs__Header(objp);
  return tcpstp->err;
#undef _CHKOK
}

/**
 * @brief   Sends a TCPROS <tt>std_msgs/Header</tt> message.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @param[in] objp
 *          Pointer to an initialized <code>struct msg__std_msgs__Header</code> object.
 * @return
 *          Error code.
 */
uros_err_t send_msg__std_msgs__Header(
  UrosTcpRosStatus *tcpstp,
  struct msg__std_msgs__Header *objp
) {
  urosAssert(tcpstp != NULL);
  urosAssert(urosConnIsValid(tcpstp->csp));
  urosAssert(objp != NULL);
#define _CHKOK { if (tcpstp->err != UROS_OK) { return tcpstp->err; } }

  urosTcpRosSendRaw(tcpstp, objp->seq); _CHKOK
  urosTcpRosSendRaw(tcpstp, objp->stamp); _CHKOK
  urosTcpRosSendString(tcpstp, &objp->frame_id); _CHKOK

  return tcpstp->err = UROS_OK;
#undef _CHKOK
}

/** @} */

/*~~~ MESSAGE: hardware_tools_msgs/ImuRaw ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Message <tt>hardware_tools_msgs/ImuRaw</tt> */
/** @{ */

/**
 * @brief   Content length of a TCPROS <tt>hardware_tools_msgs/ImuRaw</tt> message.
 *
 * @param[in,out] objp
 *          Pointer to an initialized <code>struct msg__hardware_tools_msgs__ImuRaw</code> object.
 * @return
 *          Length of the TCPROS message contents, in bytes.
 */
size_t length_msg__hardware_tools_msgs__ImuRaw(
  struct msg__hardware_tools_msgs__ImuRaw *objp
) {
  size_t length = 0;

  urosAssert(objp != NULL);

  length += length_msg__std_msgs__Header(&objp->header);
  length += (size_t)9 * sizeof(int32_t);

  return length;
}

/**
 * @brief   Initializes a TCPROS <tt>hardware_tools_msgs/ImuRaw</tt> message.
 *
 * @param[in,out] objp
 *          Pointer to an allocated <code>struct msg__hardware_tools_msgs__ImuRaw</code> object.
 * @return
 *          Error code.
 */
void init_msg__hardware_tools_msgs__ImuRaw(
  struct msg__hardware_tools_msgs__ImuRaw *objp
) {
  urosAssert(objp != NULL);

  init_msg__std_msgs__Header(&objp->header);
}

/**
 * @brief   Cleans a TCPROS <tt>hardware_tools_msgs/ImuRaw</tt> message.
 *
 * @param[in,out] objp
 *          Pointer to an initialized <code>struct msg__hardware_tools_msgs__ImuRaw</code> object, or @p NULL.
 * @return
 *          Error code.
 */
void clean_msg__hardware_tools_msgs__ImuRaw(
  struct msg__hardware_tools_msgs__ImuRaw *objp
) {
  if (objp == NULL) { return; }

  clean_msg__std_msgs__Header(&objp->header);
}

/**
 * @brief   Receives a TCPROS <tt>hardware_tools_msgs/ImuRaw</tt> message.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @param[out] objp
 *          Pointer to an initialized <code>struct msg__hardware_tools_msgs__ImuRaw</code> object.
 * @return
 *          Error code.
 */
uros_err_t recv_msg__hardware_tools_msgs__ImuRaw(
  UrosTcpRosStatus *tcpstp,
  struct msg__hardware_tools_msgs__ImuRaw *objp
) {
  urosAssert(tcpstp != NULL);
  urosAssert(urosConnIsValid(tcpstp->csp));
  urosAssert(objp != NULL);
#define _CHKOK { if (tcpstp->err != UROS_OK) { goto _error; } }

  recv_msg__std_msgs__Header(tcpstp, &objp->header); _CHKOK
  urosTcpRosRecv(tcpstp, objp->data,
                 (size_t)9 * sizeof(int32_t)); _CHKOK

  return tcpstp->err = UROS_OK;
_error:
  clean_msg__hardware_tools_msgs__ImuRaw(objp);
  return tcpstp->err;
#undef _CHKOK
}

/**
 * @brief   Sends a TCPROS <tt>hardware_tools_msgs/ImuRaw</tt> message.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @param[in] objp
 *          Pointer to an initialized <code>struct msg__hardware_tools_msgs__ImuRaw</code> object.
 * @return
 *          Error code.
 */
uros_err_t send_msg__hardware_tools_msgs__ImuRaw(
  UrosTcpRosStatus *tcpstp,
  struct msg__hardware_tools_msgs__ImuRaw *objp
) {
  urosAssert(tcpstp != NULL);
  urosAssert(urosConnIsValid(tcpstp->csp));
  urosAssert(objp != NULL);
#define _CHKOK { if (tcpstp->err != UROS_OK) { return tcpstp->err; } }

  send_msg__std_msgs__Header(tcpstp, &objp->header); _CHKOK
  urosTcpRosSend(tcpstp, objp->data,
                 (size_t)9 * sizeof(int32_t)); _CHKOK

  return tcpstp->err = UROS_OK;
#undef _CHKOK
}

/** @} */

/*~~~ MESSAGE: std_msgs/Float32 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/** @name Message <tt>std_msgs/Float32</tt> */
/** @{ */

/**
 * @brief   Content length of a TCPROS <tt>std_msgs/Float32</tt> message.
 *
 * @param[in,out] objp
 *          Pointer to an initialized <code>struct msg__std_msgs__Float32</code> object.
 * @return
 *          Length of the TCPROS message contents, in bytes.
 */
size_t length_msg__std_msgs__Float32(
  struct msg__std_msgs__Float32 *objp
) {
  size_t length = 0;

  urosAssert(objp != NULL);

  length += sizeof(float);

  (void)objp;
  return length;
}

/**
 * @brief   Initializes a TCPROS <tt>std_msgs/Float32</tt> message.
 *
 * @param[in,out] objp
 *          Pointer to an allocated <code>struct msg__std_msgs__Float32</code> object.
 * @return
 *          Error code.
 */
void init_msg__std_msgs__Float32(
  struct msg__std_msgs__Float32 *objp
) {
  urosAssert(objp != NULL);

  /* Nothing to initialize.*/
  (void)objp;
}

/**
 * @brief   Cleans a TCPROS <tt>std_msgs/Float32</tt> message.
 *
 * @param[in,out] objp
 *          Pointer to an initialized <code>struct msg__std_msgs__Float32</code> object, or @p NULL.
 * @return
 *          Error code.
 */
void clean_msg__std_msgs__Float32(
  struct msg__std_msgs__Float32 *objp
) {
  /* Nothing to clean.*/
  (void)objp;
}

/**
 * @brief   Receives a TCPROS <tt>std_msgs/Float32</tt> message.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @param[out] objp
 *          Pointer to an initialized <code>struct msg__std_msgs__Float32</code> object.
 * @return
 *          Error code.
 */
uros_err_t recv_msg__std_msgs__Float32(
  UrosTcpRosStatus *tcpstp,
  struct msg__std_msgs__Float32 *objp
) {
  urosAssert(tcpstp != NULL);
  urosAssert(urosConnIsValid(tcpstp->csp));
  urosAssert(objp != NULL);
#define _CHKOK { if (tcpstp->err != UROS_OK) { goto _error; } }

  urosTcpRosRecvRaw(tcpstp, objp->data); _CHKOK

  return tcpstp->err = UROS_OK;
_error:
  clean_msg__std_msgs__Float32(objp);
  return tcpstp->err;
#undef _CHKOK
}

/**
 * @brief   Sends a TCPROS <tt>std_msgs/Float32</tt> message.
 *
 * @param[in,out] tcpstp
 *          Pointer to a working @p UrosTcpRosStatus object.
 * @param[in] objp
 *          Pointer to an initialized <code>struct msg__std_msgs__Float32</code> object.
 * @return
 *          Error code.
 */
uros_err_t send_msg__std_msgs__Float32(
  UrosTcpRosStatus *tcpstp,
  struct msg__std_msgs__Float32 *objp
) {
  urosAssert(tcpstp != NULL);
  urosAssert(urosConnIsValid(tcpstp->csp));
  urosAssert(objp != NULL);
#define _CHKOK { if (tcpstp->err != UROS_OK) { return tcpstp->err; } }

  urosTcpRosSendRaw(tcpstp, objp->data); _CHKOK

  return tcpstp->err = UROS_OK;
#undef _CHKOK
}

/** @} */

/** @} */

/*===========================================================================*/
/* SERVICE FUNCTIONS                                                         */
/*===========================================================================*/

/** @addtogroup tcpros_srv_funcs */
/** @{ */

/* There are no service types.*/

/** @} */

/*===========================================================================*/
/* GLOBAL FUNCTIONS                                                          */
/*===========================================================================*/

/** @addtogroup tcpros_funcs */
/** @{ */

/**
 * @brief   Static TCPROS types registration.
 * @details Statically registers all the TCPROS message and service types used
 *          within this source file.
 * @note    Should be called by @p urosUserRegisterStaticMsgTypes().
 * @see     urosUserRegisterStaticMsgTypes()
 */
void urosMsgTypesRegStaticTypes(void) {

  /* MESSAGE TYPES */

  /* hardware_tools_msgs/ImuRaw */
  urosRegisterStaticMsgTypeSZ("hardware_tools_msgs/ImuRaw",
                              NULL, "91381ef4ace90e546aff80c04c74c732");

  /* std_msgs/Float32 */
  urosRegisterStaticMsgTypeSZ("std_msgs/Float32",
                              NULL, "73fcbf46b49191e672908e50842a83d4");

  /* std_msgs/Header */
  urosRegisterStaticMsgTypeSZ("std_msgs/Header",
                              NULL, "2176decaecbce78abc3b96ef049fabed");
}

/** @} */

