#ifndef _ROS_r2p_msgs_Encoder2_h
#define _ROS_r2p_msgs_Encoder2_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2p_msgs
{

  class Encoder2 : public ros::Msg
  {
    public:
      float left_delta;
      float right_delta;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_delta;
      u_left_delta.real = this->left_delta;
      *(outbuffer + offset + 0) = (u_left_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_delta);
      union {
        float real;
        uint32_t base;
      } u_right_delta;
      u_right_delta.real = this->right_delta;
      *(outbuffer + offset + 0) = (u_right_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_delta);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_delta;
      u_left_delta.base = 0;
      u_left_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_delta = u_left_delta.real;
      offset += sizeof(this->left_delta);
      union {
        float real;
        uint32_t base;
      } u_right_delta;
      u_right_delta.base = 0;
      u_right_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_delta = u_right_delta.real;
      offset += sizeof(this->right_delta);
     return offset;
    }

    const char * getType(){ return "r2p_msgs/Encoder2"; };
    const char * getMD5(){ return "4acf5cfc85b53b1f9357f2a7f535ca66"; };

  };

}
#endif