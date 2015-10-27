#ifndef _ROS_roamros_msgs_SingleTrackAckermannOdometry_h
#define _ROS_roamros_msgs_SingleTrackAckermannOdometry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace roamros_msgs
{

  class SingleTrackAckermannOdometry : public ros::Msg
  {
    public:
      float speed;
      float steer;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->steer);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->steer));
     return offset;
    }

    const char * getType(){ return "roamros_msgs/SingleTrackAckermannOdometry"; };
    const char * getMD5(){ return "64a28906316460b1514f030ee73f69e5"; };

  };

}
#endif