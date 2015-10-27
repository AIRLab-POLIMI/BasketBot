#ifndef _ROS_r2p_msgs_EncoderStamped_h
#define _ROS_r2p_msgs_EncoderStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "r2p_msgs/Encoder.h"

namespace r2p_msgs
{

  class EncoderStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      r2p_msgs::Encoder encoder;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->encoder.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->encoder.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "r2p_msgs/EncoderStamped"; };
    const char * getMD5(){ return "6d28db09933fa8a280e44044a6edf9c1"; };

  };

}
#endif