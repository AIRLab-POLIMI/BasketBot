#ifndef _ROS_SERVICE_RegisterNode_h
#define _ROS_SERVICE_RegisterNode_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

namespace heartbeat
{

static const char REGISTERNODE[] = "heartbeat/RegisterNode";

  class RegisterNodeRequest : public ros::Msg
  {
    public:
      std_msgs::String node_name;
      std_msgs::Float32 timeout;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->node_name.serialize(outbuffer + offset);
      offset += this->timeout.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->node_name.deserialize(inbuffer + offset);
      offset += this->timeout.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return REGISTERNODE; };
    const char * getMD5(){ return "cb9cd4cb67c0c3b9ac03022db365eca0"; };

  };

  class RegisterNodeResponse : public ros::Msg
  {
    public:
      bool success;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return REGISTERNODE; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class RegisterNode {
    public:
    typedef RegisterNodeRequest Request;
    typedef RegisterNodeResponse Response;
  };

}
#endif
