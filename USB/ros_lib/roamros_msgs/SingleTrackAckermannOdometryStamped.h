#ifndef _ROS_roamros_msgs_SingleTrackAckermannOdometryStamped_h
#define _ROS_roamros_msgs_SingleTrackAckermannOdometryStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "roamros_msgs/SingleTrackAckermannOdometry.h"

namespace roamros_msgs
{

  class SingleTrackAckermannOdometryStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      roamros_msgs::SingleTrackAckermannOdometry odometry;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->odometry.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->odometry.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "roamros_msgs/SingleTrackAckermannOdometryStamped"; };
    const char * getMD5(){ return "b45a0b4510a8a4a9b257b059b33c36b4"; };

  };

}
#endif