#ifndef _ROS_r2p_msgs_ImuRawStamped_h
#define _ROS_r2p_msgs_ImuRawStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "r2p_msgs/ImuRaw.h"

namespace r2p_msgs
{

  class ImuRawStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      r2p_msgs::ImuRaw imu;

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

    const char * getType(){ return "r2p_msgs/ImuRawStamped"; };
    const char * getMD5(){ return "d6823896cc6d75a6a5a20427e3fde1a0"; };

  };

}
#endif