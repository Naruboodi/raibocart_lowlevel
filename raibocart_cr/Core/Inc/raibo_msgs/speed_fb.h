#ifndef _ROS_raibo_msgs_speed_fb_h
#define _ROS_raibo_msgs_speed_fb_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace raibo_msgs
{

  class speed_fb : public ros::Msg
  {
    public:
      typedef double _bldc_rpm_sp_type;
      _bldc_rpm_sp_type bldc_rpm_sp;
      typedef double _bldc_rpm_fb_type;
      _bldc_rpm_fb_type bldc_rpm_fb;
      typedef double _bldc_mps_fb_type;
      _bldc_mps_fb_type bldc_mps_fb;
      typedef double _bldc_kmph_fb_type;
      _bldc_kmph_fb_type bldc_kmph_fb;
      typedef double _steering_angle_sp_type;
      _steering_angle_sp_type steering_angle_sp;
      typedef double _steering_angular_fb_type;
      _steering_angular_fb_type steering_angular_fb;
      typedef double _steering_rpm_fb_type;
      _steering_rpm_fb_type steering_rpm_fb;
      typedef bool _forward_motor_type;
      _forward_motor_type forward_motor;

    speed_fb():
      bldc_rpm_sp(0),
      bldc_rpm_fb(0),
      bldc_mps_fb(0),
      bldc_kmph_fb(0),
      steering_angle_sp(0),
      steering_angular_fb(0),
      steering_rpm_fb(0),
      forward_motor(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_bldc_rpm_sp;
      u_bldc_rpm_sp.real = this->bldc_rpm_sp;
      *(outbuffer + offset + 0) = (u_bldc_rpm_sp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bldc_rpm_sp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bldc_rpm_sp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bldc_rpm_sp.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_bldc_rpm_sp.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_bldc_rpm_sp.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_bldc_rpm_sp.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_bldc_rpm_sp.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->bldc_rpm_sp);
      union {
        double real;
        uint64_t base;
      } u_bldc_rpm_fb;
      u_bldc_rpm_fb.real = this->bldc_rpm_fb;
      *(outbuffer + offset + 0) = (u_bldc_rpm_fb.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bldc_rpm_fb.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bldc_rpm_fb.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bldc_rpm_fb.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_bldc_rpm_fb.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_bldc_rpm_fb.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_bldc_rpm_fb.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_bldc_rpm_fb.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->bldc_rpm_fb);
      union {
        double real;
        uint64_t base;
      } u_bldc_mps_fb;
      u_bldc_mps_fb.real = this->bldc_mps_fb;
      *(outbuffer + offset + 0) = (u_bldc_mps_fb.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bldc_mps_fb.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bldc_mps_fb.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bldc_mps_fb.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_bldc_mps_fb.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_bldc_mps_fb.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_bldc_mps_fb.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_bldc_mps_fb.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->bldc_mps_fb);
      union {
        double real;
        uint64_t base;
      } u_bldc_kmph_fb;
      u_bldc_kmph_fb.real = this->bldc_kmph_fb;
      *(outbuffer + offset + 0) = (u_bldc_kmph_fb.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bldc_kmph_fb.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bldc_kmph_fb.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bldc_kmph_fb.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_bldc_kmph_fb.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_bldc_kmph_fb.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_bldc_kmph_fb.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_bldc_kmph_fb.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->bldc_kmph_fb);
      union {
        double real;
        uint64_t base;
      } u_steering_angle_sp;
      u_steering_angle_sp.real = this->steering_angle_sp;
      *(outbuffer + offset + 0) = (u_steering_angle_sp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering_angle_sp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steering_angle_sp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steering_angle_sp.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_steering_angle_sp.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_steering_angle_sp.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_steering_angle_sp.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_steering_angle_sp.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->steering_angle_sp);
      union {
        double real;
        uint64_t base;
      } u_steering_angular_fb;
      u_steering_angular_fb.real = this->steering_angular_fb;
      *(outbuffer + offset + 0) = (u_steering_angular_fb.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering_angular_fb.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steering_angular_fb.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steering_angular_fb.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_steering_angular_fb.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_steering_angular_fb.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_steering_angular_fb.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_steering_angular_fb.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->steering_angular_fb);
      union {
        double real;
        uint64_t base;
      } u_steering_rpm_fb;
      u_steering_rpm_fb.real = this->steering_rpm_fb;
      *(outbuffer + offset + 0) = (u_steering_rpm_fb.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering_rpm_fb.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steering_rpm_fb.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steering_rpm_fb.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_steering_rpm_fb.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_steering_rpm_fb.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_steering_rpm_fb.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_steering_rpm_fb.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->steering_rpm_fb);
      union {
        bool real;
        uint8_t base;
      } u_forward_motor;
      u_forward_motor.real = this->forward_motor;
      *(outbuffer + offset + 0) = (u_forward_motor.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->forward_motor);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_bldc_rpm_sp;
      u_bldc_rpm_sp.base = 0;
      u_bldc_rpm_sp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bldc_rpm_sp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bldc_rpm_sp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bldc_rpm_sp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_bldc_rpm_sp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_bldc_rpm_sp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_bldc_rpm_sp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_bldc_rpm_sp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->bldc_rpm_sp = u_bldc_rpm_sp.real;
      offset += sizeof(this->bldc_rpm_sp);
      union {
        double real;
        uint64_t base;
      } u_bldc_rpm_fb;
      u_bldc_rpm_fb.base = 0;
      u_bldc_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bldc_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bldc_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bldc_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_bldc_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_bldc_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_bldc_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_bldc_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->bldc_rpm_fb = u_bldc_rpm_fb.real;
      offset += sizeof(this->bldc_rpm_fb);
      union {
        double real;
        uint64_t base;
      } u_bldc_mps_fb;
      u_bldc_mps_fb.base = 0;
      u_bldc_mps_fb.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bldc_mps_fb.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bldc_mps_fb.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bldc_mps_fb.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_bldc_mps_fb.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_bldc_mps_fb.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_bldc_mps_fb.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_bldc_mps_fb.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->bldc_mps_fb = u_bldc_mps_fb.real;
      offset += sizeof(this->bldc_mps_fb);
      union {
        double real;
        uint64_t base;
      } u_bldc_kmph_fb;
      u_bldc_kmph_fb.base = 0;
      u_bldc_kmph_fb.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bldc_kmph_fb.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bldc_kmph_fb.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bldc_kmph_fb.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_bldc_kmph_fb.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_bldc_kmph_fb.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_bldc_kmph_fb.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_bldc_kmph_fb.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->bldc_kmph_fb = u_bldc_kmph_fb.real;
      offset += sizeof(this->bldc_kmph_fb);
      union {
        double real;
        uint64_t base;
      } u_steering_angle_sp;
      u_steering_angle_sp.base = 0;
      u_steering_angle_sp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering_angle_sp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steering_angle_sp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steering_angle_sp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_steering_angle_sp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_steering_angle_sp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_steering_angle_sp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_steering_angle_sp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->steering_angle_sp = u_steering_angle_sp.real;
      offset += sizeof(this->steering_angle_sp);
      union {
        double real;
        uint64_t base;
      } u_steering_angular_fb;
      u_steering_angular_fb.base = 0;
      u_steering_angular_fb.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering_angular_fb.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steering_angular_fb.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steering_angular_fb.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_steering_angular_fb.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_steering_angular_fb.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_steering_angular_fb.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_steering_angular_fb.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->steering_angular_fb = u_steering_angular_fb.real;
      offset += sizeof(this->steering_angular_fb);
      union {
        double real;
        uint64_t base;
      } u_steering_rpm_fb;
      u_steering_rpm_fb.base = 0;
      u_steering_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steering_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steering_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_steering_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_steering_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_steering_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_steering_rpm_fb.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->steering_rpm_fb = u_steering_rpm_fb.real;
      offset += sizeof(this->steering_rpm_fb);
      union {
        bool real;
        uint8_t base;
      } u_forward_motor;
      u_forward_motor.base = 0;
      u_forward_motor.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->forward_motor = u_forward_motor.real;
      offset += sizeof(this->forward_motor);
     return offset;
    }

    virtual const char * getType() override { return "raibo_msgs/speed_fb"; };
    virtual const char * getMD5() override { return "960d1a8f0ecee7e59a8c5dceb958501e"; };

  };

}
#endif
