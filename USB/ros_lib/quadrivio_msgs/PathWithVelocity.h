#ifndef _ROS_quadrivio_msgs_PathWithVelocity_h
#define _ROS_quadrivio_msgs_PathWithVelocity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Path.h"

namespace quadrivio_msgs
{

  class PathWithVelocity : public ros::Msg
  {
    public:
      std_msgs::Header header;
      nav_msgs::Path path;
      uint8_t velocities_length;
      float st_velocities;
      float * velocities;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->path.serialize(outbuffer + offset);
      *(outbuffer + offset++) = velocities_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < velocities_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->velocities[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->path.deserialize(inbuffer + offset);
      uint8_t velocities_lengthT = *(inbuffer + offset++);
      if(velocities_lengthT > velocities_length)
        this->velocities = (float*)realloc(this->velocities, velocities_lengthT * sizeof(float));
      offset += 3;
      velocities_length = velocities_lengthT;
      for( uint8_t i = 0; i < velocities_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_velocities));
        memcpy( &(this->velocities[i]), &(this->st_velocities), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "quadrivio_msgs/PathWithVelocity"; };
    const char * getMD5(){ return "a0466898388de0c5be5d2fed0c083dc0"; };

  };

}
#endif