/**
 * @file    urosMsgTypes.h
 * @author  TODO
 *
 * @brief   TCPROS message and service descriptors.
 */

#ifndef _UROSMSGTYPES_H_
#define _UROSMSGTYPES_H_

/*===========================================================================*/
/* HEADER FILES                                                              */
/*===========================================================================*/

#include <urosTcpRos.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/*  MESSAGE TYPES                                                            */
/*===========================================================================*/

/** @addtogroup tcpros_msg_types */
/** @{ */

/*~~~ MESSAGE: std_msgs/Header ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/**
 * @brief   TCPROS <tt>std_msgs/Header</tt> message descriptor.
 * @details MD5 sum: <tt>2176decaecbce78abc3b96ef049fabed</tt>.
 */
struct msg__std_msgs__Header {
  uint32_t      seq;
  uros_time_t   stamp;
  UrosString    frame_id;
};

/*~~~ MESSAGE: hardware_tools_msgs/ImuRaw ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/**
 * @brief   TCPROS <tt>hardware_tools_msgs/ImuRaw</tt> message descriptor.
 * @details MD5 sum: <tt>91381ef4ace90e546aff80c04c74c732</tt>.
 */
struct msg__hardware_tools_msgs__ImuRaw {
  struct msg__std_msgs__Header  header;
  int32_t                       data[9];
};

/*~~~ MESSAGE: std_msgs/Float32 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/**
 * @brief   TCPROS <tt>std_msgs/Float32</tt> message descriptor.
 * @details MD5 sum: <tt>73fcbf46b49191e672908e50842a83d4</tt>.
 */
struct msg__std_msgs__Float32 {
  float data;
};

/** @} */

/*===========================================================================*/
/* SERVICE TYPES                                                             */
/*===========================================================================*/

/** @addtogroup tcpros_srv_types */
/** @{ */

/* There are no service types.*/

/** @} */

/*===========================================================================*/
/* MESSAGE CONSTANTS                                                         */
/*===========================================================================*/

/** @addtogroup tcpros_msg_consts */
/** @{ */

/* There are no message costants.*/

/** @} */

/*===========================================================================*/
/* SERVICE CONSTANTS                                                         */
/*===========================================================================*/

/** @addtogroup tcpros_srv_consts */
/** @{ */

/* There are no service costants.*/

/** @} */

/*===========================================================================*/
/* MESSAGE PROTOTYPES                                                        */
/*===========================================================================*/

/*~~~ MESSAGE: std_msgs/Header ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

size_t length_msg__std_msgs__Header(
  struct msg__std_msgs__Header *objp
);
void init_msg__std_msgs__Header(
  struct msg__std_msgs__Header *objp
);
void clean_msg__std_msgs__Header(
  struct msg__std_msgs__Header *objp
);
uros_err_t recv_msg__std_msgs__Header(
  UrosTcpRosStatus *tcpstp,
  struct msg__std_msgs__Header *objp
);
uros_err_t send_msg__std_msgs__Header(
  UrosTcpRosStatus *tcpstp,
  struct msg__std_msgs__Header *objp
);

/*~~~ MESSAGE: hardware_tools_msgs/ImuRaw ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

size_t length_msg__hardware_tools_msgs__ImuRaw(
  struct msg__hardware_tools_msgs__ImuRaw *objp
);
void init_msg__hardware_tools_msgs__ImuRaw(
  struct msg__hardware_tools_msgs__ImuRaw *objp
);
void clean_msg__hardware_tools_msgs__ImuRaw(
  struct msg__hardware_tools_msgs__ImuRaw *objp
);
uros_err_t recv_msg__hardware_tools_msgs__ImuRaw(
  UrosTcpRosStatus *tcpstp,
  struct msg__hardware_tools_msgs__ImuRaw *objp
);
uros_err_t send_msg__hardware_tools_msgs__ImuRaw(
  UrosTcpRosStatus *tcpstp,
  struct msg__hardware_tools_msgs__ImuRaw *objp
);

/*~~~ MESSAGE: std_msgs/Float32 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

size_t length_msg__std_msgs__Float32(
  struct msg__std_msgs__Float32 *objp
);
void init_msg__std_msgs__Float32(
  struct msg__std_msgs__Float32 *objp
);
void clean_msg__std_msgs__Float32(
  struct msg__std_msgs__Float32 *objp
);
uros_err_t recv_msg__std_msgs__Float32(
  UrosTcpRosStatus *tcpstp,
  struct msg__std_msgs__Float32 *objp
);
uros_err_t send_msg__std_msgs__Float32(
  UrosTcpRosStatus *tcpstp,
  struct msg__std_msgs__Float32 *objp
);

/*===========================================================================*/
/* SERVICE PROTOTYPES                                                        */
/*===========================================================================*/

/* There are no service types.*/

/*===========================================================================*/
/* GLOBAL PROTOTYPES                                                         */
/*===========================================================================*/

void urosMsgTypesRegStaticTypes(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _UROSMSGTYPES_H_ */

