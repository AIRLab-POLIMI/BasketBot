#ifndef _ROS_quadrivio_msgs_OtherValues_h
#define _ROS_quadrivio_msgs_OtherValues_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace quadrivio_msgs
{

  class OtherValues : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float steerCurrentSP;
      float LLT;
      float FLT;
      float stateMachine;
      float steerCurrentPV;
      float batteryVoltage;
      float strokePVFL;
      float strokePVFR;
      float strokePVRL;
      float strokePVRR;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->steerCurrentSP);
      offset += serializeAvrFloat64(outbuffer + offset, this->LLT);
      offset += serializeAvrFloat64(outbuffer + offset, this->FLT);
      offset += serializeAvrFloat64(outbuffer + offset, this->stateMachine);
      offset += serializeAvrFloat64(outbuffer + offset, this->steerCurrentPV);
      offset += serializeAvrFloat64(outbuffer + offset, this->batteryVoltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->strokePVFL);
      offset += serializeAvrFloat64(outbuffer + offset, this->strokePVFR);
      offset += serializeAvrFloat64(outbuffer + offset, this->strokePVRL);
      offset += serializeAvrFloat64(outbuffer + offset, this->strokePVRR);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->steerCurrentSP));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->LLT));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->FLT));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->stateMachine));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->steerCurrentPV));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->batteryVoltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->strokePVFL));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->strokePVFR));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->strokePVRL));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->strokePVRR));
     return offset;
    }

    const char * getType(){ return "quadrivio_msgs/OtherValues"; };
    const char * getMD5(){ return "a671ac4c45dc8979d41391920669ebae"; };

  };

}
#endif