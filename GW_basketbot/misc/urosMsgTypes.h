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

/*~~~ MESSAGE: geometry_msgs/Vector3 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/**
 * @brief   TCPROS <tt>geometry_msgs/Vector3</tt> message descriptor.
 * @details MD5 sum: <tt>4a842b65f413084dc2b10fb484ea7f17</tt>.
 */
struct msg__geometry_msgs__Vector3 {
  double    x;
  double    y;
  double    z;
};

/*~~~ MESSAGE: geometry_msgs/Twist ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/**
 * @brief   TCPROS <tt>geometry_msgs/Twist</tt> message descriptor.
 * @details MD5 sum: <tt>9f195f881246fdfa2798d1d3eebca84a</tt>.
 */
struct msg__geometry_msgs__Twist {
  struct msg__geometry_msgs__Vector3    linear;
  struct msg__geometry_msgs__Vector3    angular;
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

/*~~~ MESSAGE: geometry_msgs/Vector3 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

size_t length_msg__geometry_msgs__Vector3(
  struct msg__geometry_msgs__Vector3 *objp
);
void init_msg__geometry_msgs__Vector3(
  struct msg__geometry_msgs__Vector3 *objp
);
void clean_msg__geometry_msgs__Vector3(
  struct msg__geometry_msgs__Vector3 *objp
);
uros_err_t recv_msg__geometry_msgs__Vector3(
  UrosTcpRosStatus *tcpstp,
  struct msg__geometry_msgs__Vector3 *objp
);
uros_err_t send_msg__geometry_msgs__Vector3(
  UrosTcpRosStatus *tcpstp,
  struct msg__geometry_msgs__Vector3 *objp
);

/*~~~ MESSAGE: geometry_msgs/Twist ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

size_t length_msg__geometry_msgs__Twist(
  struct msg__geometry_msgs__Twist *objp
);
void init_msg__geometry_msgs__Twist(
  struct msg__geometry_msgs__Twist *objp
);
void clean_msg__geometry_msgs__Twist(
  struct msg__geometry_msgs__Twist *objp
);
uros_err_t recv_msg__geometry_msgs__Twist(
  UrosTcpRosStatus *tcpstp,
  struct msg__geometry_msgs__Twist *objp
);
uros_err_t send_msg__geometry_msgs__Twist(
  UrosTcpRosStatus *tcpstp,
  struct msg__geometry_msgs__Twist *objp
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

