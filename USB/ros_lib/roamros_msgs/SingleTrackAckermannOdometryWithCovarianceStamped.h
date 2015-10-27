#ifndef _ROS_roamros_msgs_SingleTrackAckermannOdometryWithCovarianceStamped_h
#define _ROS_roamros_msgs_SingleTrackAckermannOdometryWithCovarianceStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "roamros_msgs/SingleTrackAckermannOdometryWithCovariance.h"

namespace roamros_msgs
{

  class SingleTrackAckermannOdometryWithCovarianceStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      roamros_msgs::SingleTrackAckermannOdometryWithCovariance odometry;

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

    const char * getType(){ return "roamros_msgs/SingleTrackAckermannOdometryWithCovarianceStamped"; };
    const char * getMD5(){ return "4617f709dff3d2940b6c42587f2839d4"; };

  };

}
#endif