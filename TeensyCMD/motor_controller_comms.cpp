#include "motor_controller_comms.h"
#include "../HemoMotor/motor_define.h"
// #include <Arduino.h>
#include "stdint.h"

float abs(float f) {
  if (f < 0) {
    return -1 * f;
  }

  return f;
}

void make_packet_vels(unsigned char *buf, float vel_l, float vel_r) {
  buf[0] = PACKET_START;
  buf[PACKET_SIZE-1] = PACKET_END;

  float spd_l = abs(vel_l);
  float spd_r = abs(vel_r);
  if(spd_l > .02) {
    buf[1] = vel_l > 0.0 ? MOTOR_FWD : MOTOR_BWD;
    buf[2] = (int) spd_l;
    buf[3] = (int)((spd_l - buf[1]) * 100);
  }
  if(spd_r > .02) {
    buf[4] = vel_r > 0.0 ? MOTOR_FWD : MOTOR_BWD;
    buf[5] = (int) spd_r;
    buf[6] = (int)((spd_r - buf[4]) * 100);
  }
  else {
    buf[1] = buf[2] = buf[3] = buf[4] = buf[5] = buf[6] = 0;
  }
  //Serial.println("ret");
}