#ifndef _ROS_quadrivio_msgs_SetPoint_h
#define _ROS_quadrivio_msgs_SetPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace quadrivio_msgs
{

  class SetPoint : public ros::Msg
  {
    public:
      float speed;
      float steer;
      float brake;
      uint8_t source;
      enum { AUTO =  0 };
      enum { JOYPAD =  1 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->steer);
      offset += serializeAvrFloat64(outbuffer + offset, this->brake);
      *(outbuffer + offset + 0) = (this->source >> (8 * 0)) & 0xFF;
      offset += sizeof(this->source);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->steer));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->brake));
      this->source =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->source);
     return offset;
    }

    const char * getType(){ return "quadrivio_msgs/SetPoint"; };
    const char * getMD5(){ return "8bd90c78c2a3e2624e4cb35d1fc6370f"; };

  };

}
#endif