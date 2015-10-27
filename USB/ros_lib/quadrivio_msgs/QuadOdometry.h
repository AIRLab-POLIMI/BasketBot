#ifndef _ROS_quadrivio_msgs_QuadOdometry_h
#define _ROS_quadrivio_msgs_QuadOdometry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace quadrivio_msgs
{

  class QuadOdometry : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float speed;
      float steer;
      float brake;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->steer);
      offset += serializeAvrFloat64(outbuffer + offset, this->brake);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->steer));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->brake));
     return offset;
    }

    const char * getType(){ return "quadrivio_msgs/QuadOdometry"; };
    const char * getMD5(){ return "c1fa88c9ff51c2e4d979f6c99394901b"; };

  };

}
#endif