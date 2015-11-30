// gcc -o a.out serialDebug.cpp TeensyCMD/motor_controller_comms.cpp SerialMessage/SerialMessage.cpp PiSerial/PiSerial.cpp HemoMotor/motor_define.h 


#include "HemoMotor/motor_define.h"

#include "SerialMessage/SerialMessage.h"
#include "PiSerial/PiSerial.h"
#include <unistd.h>

unsigned char rx_buf[PACKET_SIZE];
unsigned char tx_buf[PACKET_SIZE+1];

char buf[PACKET_SIZE];

void send_command(int type, unsigned char *b, int len) {
	tx_buf[0] = PACKET_START;
	tx_buf[1] = type;
	for (int i = 2; i < PACKET_SIZE - 1; i++) {
		if (i-2 < len) { 
			tx_buf[i] = b[i-2];	
		} else {
			tx_buf[i] = 0;
		}
	}
	tx_buf[PACKET_SIZE - 1] = PACKET_END;
	tx_buf[PACKET_SIZE] = '\0';
	sendPiSerial(tx_buf, PACKET_SIZE+1);
}

void send_command(int type, float vel) {
	unsigned char b[4];
	int off = type == TURN_MSG ? 1 : 0;
	int dir;
	if (type == TURN_MSG) {
		if (vel < 0) {
			vel = -vel;
			dir = i_to_c(DIR_LEFT);
		} else {
			dir = i_to_c(DIR_RIGHT);
		}
		b[0] = dir;
	} else {
		b[3] = 0;
	}

	if (vel < 0) {
		vel = -vel;
	}

	b[0+off] = ((int)vel)/10;
	b[1+off] = ((int)vel)%10;
	b[2+off] = vel-(int)vel;

	send_command(type, b, 4);
}

void send_command(int type) {
	send_command(type, '\0', 0);
}

int main(int ac, char** av) {

	initPiSerial();
	setbuf(stdout, NULL);

	// usleep(1000000);

	// send_command(BWD_MSG, 2.0);

	// usleep(1000000);

	// send_command(TURN_MSG, 2.0);

	// usleep(1000000);

	// send_command(TURN_MSG, -2.0);

	// usleep(1000000);

	// send_command(BRK_MSG);

	while (true) {

		// sleep 100ms to avoid overflowing UART buffer
		usleep(100000);
		send_command(FWD_MSG, 2.0);

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