#ifndef _ROS_r2p_msgs_ImuRaw_h
#define _ROS_r2p_msgs_ImuRaw_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "r2p_msgs/Vector3_32.h"

namespace r2p_msgs
{

  class ImuRaw : public ros::Msg
  {
    public:
      r2p_msgs::Vector3_32 angular_velocity;
      r2p_msgs::Vector3_32 linear_acceleration;
      r2p_msgs::Vector3_32 magnetic_field;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->angular_velocity.serialize(outbuffer + offset);
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      offset += this->magnetic_field.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      offset += this->magnetic_field.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "r2p_msgs/ImuRaw"; };
    const char * getMD5(){ return "038fa76b07790b50cb414643eae47fb9"; };

  };

}
#endif