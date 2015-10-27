#ifndef _ROS_heartbeat_State_h
#define _ROS_heartbeat_State_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace heartbeat
{

  class State : public ros::Msg
  {
    public:
      uint8_t value;
      enum { HALT = 0 };
      enum { MANUAL = 1 };
      enum { SAFE = 2 };
      enum { ASSISTED = 3 };
      enum { AUTO = 4 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->value >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->value =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->value);
     return offset;
    }

    const char * getType(){ return "heartbeat/State"; };
    const char * getMD5(){ return "8894b727e9ea5dc912a2ffdad059695b"; };

  };

}
#endif