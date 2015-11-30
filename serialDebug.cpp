// gcc -o a.out serialDebug.cpp TeensyCMD/motor_controller_comms.cpp SerialMessage/SerialMessage.cpp PiSerial/PiSerial.cpp HemoMotor/motor_define.h 


#include "HemoMotor/motor_define.h"

#include "SerialMessage/SerialMessage.h"
#include "PiSerial/PiSerial.h"

unsigned char rx_buf[PACKET_SIZE];
unsigned char tx_buf[PACKET_SIZE];

int main(int ac, char** av) {

	initPiSerial();
	setbuf(stdout, NULL);

	while (true) {

		int r_len = readPiSerial(rx_buf, PACKET_SIZE);
		if (r_len > 0) {
			// printf("Len: %d\nMsg: ", r_len);
			for (int i = 0; i < r_len; i++) {
				printf("%c", (char)rx_buf[i]);
			}
			// printf("\r\n");
		}
	}
}