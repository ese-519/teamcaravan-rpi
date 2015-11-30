#ifndef __MC_HAMMER_H__
#define __MC_HAMMER_H__

// #include "Arduino.h"
#include "stdint.h"

#define PACKET_SIZE 10
#define PACKET_START 2   // STX
#define PACKET_END 3     // ETX

float abs(float f);

void make_packet_vels(unsigned char *buf, float vel_l, float vel_r);

#endif