#ifndef _ROS_r2p_msgs_Encoder_h
#define _ROS_r2p_msgs_Encoder_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2p_msgs
{

  class Encoder : public ros::Msg
  {
    public:
      float delta;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_delta;
      u_delta.real = this->delta;
      *(outbuffer + offset + 0) = (u_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delta);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_delta;
      u_delta.base = 0;
      u_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->delta = u_delta.real;
      offset += sizeof(this->delta);
     return offset;
    }

    const char * getType(){ return "r2p_msgs/Encoder"; };
    const char * getMD5(){ return "b808a22d9acab1efdc64ddcf9bf82dfc"; };

  };

}
#endif