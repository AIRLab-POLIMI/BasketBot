#ifndef _ROS_roamros_msgs_SingleTrackAckermannOdometryWithCovariance_h
#define _ROS_roamros_msgs_SingleTrackAckermannOdometryWithCovariance_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "roamros_msgs/SingleTrackAckermannOdometry.h"

namespace roamros_msgs
{

  class SingleTrackAckermannOdometryWithCovariance : public ros::Msg
  {
    public:
      roamros_msgs::SingleTrackAckermannOdometry odometry;
      float covariance[36];

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->odometry.serialize(outbuffer + offset);
      for( uint8_t i = 0; i < 36; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->odometry.deserialize(inbuffer + offset);
      for( uint8_t i = 0; i < 36; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->covariance[i]));
      }
     return offset;
    }

    const char * getType(){ return "roamros_msgs/SingleTrackAckermannOdometryWithCovariance"; };
    const char * getMD5(){ return "92ff8aff0a19ac0a2c2848ccb44fa072"; };

  };

}
#endif