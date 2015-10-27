#ifndef _ROS_SERVICE_SetState_h
#define _ROS_SERVICE_SetState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "heartbeat/State.h"

namespace heartbeat
{

static const char SETSTATE[] = "heartbeat/SetState";

  class SetStateRequest : public ros::Msg
  {
    public:
      heartbeat::State from;
      heartbeat::State to;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->from.serialize(outbuffer + offset);
      offset += this->to.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->from.deserialize(inbuffer + offset);
      offset += this->to.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETSTATE; };
    const char * getMD5(){ return "ffb6ce256ebe2e0101128ddc43009e81"; };

  };

  class SetStateResponse : public ros::Msg
  {
    public:
      heartbeat::State current;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->current.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->current.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETSTATE; };
    const char * getMD5(){ return "57ae95ed1622f23bfc6d6bf0209e104d"; };

  };

  class SetState {
    public:
    typedef SetStateRequest Request;
    typedef SetStateResponse Response;
  };

}
#endif
