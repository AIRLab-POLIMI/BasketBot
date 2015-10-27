#ifndef _ROS_r2p_msgs_Proximity_h
#define _ROS_r2p_msgs_Proximity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2p_msgs
{

  class Proximity : public ros::Msg
  {
    public:
      float proximities[8];
      enum { NUM_SENSORS =  8      };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 8; i++){
      union {
        float real;
        uint32_t base;
      } u_proximitiesi;
      u_proximitiesi.real = this->proximities[i];
      *(outbuffer + offset + 0) = (u_proximitiesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_proximitiesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_proximitiesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_proximitiesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->proximities[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 8; i++){
      union {
        float real;
        uint32_t base;
      } u_proximitiesi;
      u_proximitiesi.base = 0;
      u_proximitiesi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_proximitiesi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_proximitiesi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_proximitiesi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->proximities[i] = u_proximitiesi.real;
      offset += sizeof(this->proximities[i]);
      }
     return offset;
    }

    const char * getType(){ return "r2p_msgs/Proximity"; };
    const char * getMD5(){ return "1c2f713873d56812bdabe757ca4b973f"; };

  };

}
#endif