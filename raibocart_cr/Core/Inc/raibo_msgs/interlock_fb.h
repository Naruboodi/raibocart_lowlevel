#ifndef _ROS_raibo_msgs_interlock_fb_h
#define _ROS_raibo_msgs_interlock_fb_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace raibo_msgs
{

  class interlock_fb : public ros::Msg
  {
    public:
      typedef bool _Motor_Break_type;
      _Motor_Break_type Motor_Break;
      typedef bool _Morot_Forward_type;
      _Morot_Forward_type Morot_Forward;

    interlock_fb():
      Motor_Break(0),
      Morot_Forward(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_Motor_Break;
      u_Motor_Break.real = this->Motor_Break;
      *(outbuffer + offset + 0) = (u_Motor_Break.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Motor_Break);
      union {
        bool real;
        uint8_t base;
      } u_Morot_Forward;
      u_Morot_Forward.real = this->Morot_Forward;
      *(outbuffer + offset + 0) = (u_Morot_Forward.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Morot_Forward);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_Motor_Break;
      u_Motor_Break.base = 0;
      u_Motor_Break.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Motor_Break = u_Motor_Break.real;
      offset += sizeof(this->Motor_Break);
      union {
        bool real;
        uint8_t base;
      } u_Morot_Forward;
      u_Morot_Forward.base = 0;
      u_Morot_Forward.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->Morot_Forward = u_Morot_Forward.real;
      offset += sizeof(this->Morot_Forward);
     return offset;
    }

    virtual const char * getType() override { return "raibo_msgs/interlock_fb"; };
    virtual const char * getMD5() override { return "22088b34e602d6c46f96a8f40388502d"; };

  };

}
#endif
