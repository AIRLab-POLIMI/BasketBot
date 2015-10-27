#ifndef _ROS_r2p_msgs_PidParameters_h
#define _ROS_r2p_msgs_PidParameters_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2p_msgs
{

  class PidParameters : public ros::Msg
  {
    public:
      float k;
      float ti;
      float td;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_k;
      u_k.real = this->k;
      *(outbuffer + offset + 0) = (u_k.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_k.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_k.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_k.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->k);
      union {
        float real;
        uint32_t base;
      } u_ti;
      u_ti.real = this->ti;
      *(outbuffer + offset + 0) = (u_ti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ti);
      union {
        float real;
        uint32_t base;
      } u_td;
      u_td.real = this->td;
      *(outbuffer + offset + 0) = (u_td.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_td.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_td.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_td.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->td);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_k;
      u_k.base = 0;
      u_k.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_k.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_k.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_k.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->k = u_k.real;
      offset += sizeof(this->k);
      union {
        float real;
        uint32_t base;
      } u_ti;
      u_ti.base = 0;
      u_ti.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ti.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ti.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ti.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ti = u_ti.real;
      offset += sizeof(this->ti);
      union {
        float real;
        uint32_t base;
      } u_td;
      u_td.base = 0;
      u_td.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_td.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_td.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_td.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->td = u_td.real;
      offset += sizeof(this->td);
     return offset;
    }

    const char * getType(){ return "r2p_msgs/PidParameters"; };
    const char * getMD5(){ return "5845946b7961ba4a5b9b996ece05c35b"; };

  };

}
#endif