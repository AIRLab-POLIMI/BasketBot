#ifndef _ROS_r2p_msgs_ImuStamped_h
#define _ROS_r2p_msgs_ImuStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "r2p_msgs/Imu.h"

namespace r2p_msgs
{

  class ImuStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      r2p_msgs::Imu imu;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->imu.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->imu.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "r2p_msgs/ImuStamped"; };
    const char * getMD5(){ return "05a1f23108d3c9b7050892038c5f5faa"; };

  };

}
#endif