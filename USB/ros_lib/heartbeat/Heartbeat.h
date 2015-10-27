#ifndef _ROS_heartbeat_Heartbeat_h
#define _ROS_heartbeat_Heartbeat_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"

namespace heartbeat
{

  class Heartbeat : public ros::Msg
  {
    public:
      std_msgs::String node_name;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->node_name.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->node_name.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "heartbeat/Heartbeat"; };
    const char * getMD5(){ return "cd38a069af211501ce9a41a4b1793b53"; };

  };

}
#endif