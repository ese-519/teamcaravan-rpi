#ifndef __PISERIAL_H__
#define __PISERIAL_H__

#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

void initPiSerial();

int sendPiSerial(unsigned char *buf, int len);

int readPiSerial(unsigned char* buf, int len);




#endif