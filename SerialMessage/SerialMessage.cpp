// #include <motor_define.h>
#include "../TeensyCMD/motor_controller_comms.h"
#include "SerialMessage.h"

bool is_numeric(unsigned char c) {
	return c >= 48 && c <= 57;
}

int c_to_i(unsigned char c) {
	return c - '0';
}

unsigned char i_to_c(int i) {
	return i + '0';
}

int s_to_i(unsigned char* s, int l) {
	int res = 0;
	for (int i = 0; i < l; i++) {
		if (is_numeric(s[i])) {
			res += c_to_i(s[i]);
		}
		res *= 10;
	}

	return res;
}

void make_packet(char *buf, const char *msg, int len) {
  buf[0] = PACKET_START;
  buf[PACKET_SIZE-1] = PACKET_END;

  buf[1] = INFO_MSG;

  for (int i = 2; i < len && i < PACKET_SIZE-1; i++) {
  	buf[i] = msg[i-2];
  }  
  
}