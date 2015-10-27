#ifndef _ROS_r2p_msgs_Led_h
#define _ROS_r2p_msgs_Led_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2p_msgs
{

  class Led : public ros::Msg
  {
    public:
      uint8_t led;
      uint8_t value;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->led >> (8 * 0)) & 0xFF;
      offset += sizeof(this->led);
      *(outbuffer + offset + 0) = (this->value >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->led =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->led);
      this->value =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->value);
     return offset;
    }

    const char * getType(){ return "r2p_msgs/Led"; };
    const char * getMD5(){ return "21a32d72a04d37c1add2e9d9fe9b645e"; };

  };

}
#endif