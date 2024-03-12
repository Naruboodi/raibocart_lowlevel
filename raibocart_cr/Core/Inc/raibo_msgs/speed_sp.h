#ifndef _ROS_raibo_msgs_speed_sp_h
#define _ROS_raibo_msgs_speed_sp_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace raibo_msgs
{

  class speed_sp : public ros::Msg
  {
    public:
      typedef double _bldc_speed_sp_type;
      _bldc_speed_sp_type bldc_speed_sp;
      typedef double _steering_speed_sp_type;
      _steering_speed_sp_type steering_speed_sp;

    speed_sp():
      bldc_speed_sp(0),
      steering_speed_sp(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_bldc_speed_sp;
      u_bldc_speed_sp.real = this->bldc_speed_sp;
      *(outbuffer + offset + 0) = (u_bldc_speed_sp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bldc_speed_sp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bldc_speed_sp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bldc_speed_sp.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_bldc_speed_sp.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_bldc_speed_sp.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_bldc_speed_sp.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_bldc_speed_sp.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->bldc_speed_sp);
      union {
        double real;
        uint64_t base;
      } u_steering_speed_sp;
      u_steering_speed_sp.real = this->steering_speed_sp;
      *(outbuffer + offset + 0) = (u_steering_speed_sp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering_speed_sp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steering_speed_sp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steering_speed_sp.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_steering_speed_sp.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_steering_speed_sp.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_steering_speed_sp.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_steering_speed_sp.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->steering_speed_sp);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_bldc_speed_sp;
      u_bldc_speed_sp.base = 0;
      u_bldc_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bldc_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bldc_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bldc_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_bldc_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_bldc_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_bldc_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_bldc_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->bldc_speed_sp = u_bldc_speed_sp.real;
      offset += sizeof(this->bldc_speed_sp);
      union {
        double real;
        uint64_t base;
      } u_steering_speed_sp;
      u_steering_speed_sp.base = 0;
      u_steering_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steering_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steering_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_steering_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_steering_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_steering_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_steering_speed_sp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->steering_speed_sp = u_steering_speed_sp.real;
      offset += sizeof(this->steering_speed_sp);
     return offset;
    }

    virtual const char * getType() override { return "raibo_msgs/speed_sp"; };
    virtual const char * getMD5() override { return "c9a4a5c0478b2962a50634087c3b282f"; };

  };

}
#endif
