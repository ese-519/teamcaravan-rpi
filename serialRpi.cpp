//Source: http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart


#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

int main(int ac, char** av) {

  //-------------------------
	//----- SETUP USART 0 -----
	//-------------------------
	//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
	int uart0_filestream = -1;
	
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    return 0;
	}
  else
  {
    printf("UART opened...\n");
  }
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
 
 while (true) {
 
 //----- TX BYTES -----
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;
	
	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = 'a';
	*p_tx_buffer++ = 'b';
	*p_tx_buffer++ = 'c';
	*p_tx_buffer++ = 'd';
	*p_tx_buffer++ = 'e';
	
	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
		if (count < 0)
		{
			printf("UART TX error\n");
		}
	}
 
 
 
 //----- CHECK FOR ANY RX BYTES -----
	if (uart0_filestream != -1)
	{
     unsigned char rx_char[1];
     unsigned char lidar_packet[22];
     int rx_len = 0;
     int rx_read = 0;
     while (rx_len <= 0){// || rx_char[0] != 0xFA) {
       rx_len = read(uart0_filestream, (void*)rx_char, 1); // Read one char, until we find the start bit
     }
	printf("Read: %u\r\n", (unsigned char)rx_char[0]);
     /*
     lidar_packet[rx_read] = (unsigned char)rx_char[0];
     rx_read++;
     
     while (rx_read < 22) {
       unsigned char rx_buffer[256];
       int rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
       int idx = 0;
       while (rx_length > 0) {
         lidar_packet[rx_read] = rx_buffer[idx];
         rx_read++;
         idx++;
         rx_length--;
         if (rx_read == 22 && rx_length > 0) {
           printf("%i bytes read : ", rx_read);
           int i = 0;
           for (i = 0; i < rx_read; i++) {
             //printf("%hhu ", lidar_packet[i]);
             printf("%x ", lidar_packet[i]);
           }
           printf("\n");
           
           // assume next is start, but get it if not...
           while (rx_buffer[idx] != 0xFA && rx_length > 0) {
             printf("WHOOPS\n");
             idx++;
             rx_length--;
           }
           
           rx_read = 0;
           lidar_packet[rx_read] = rx_buffer[idx];
           rx_read++;
            
         }
       }
     }
      printf("%i bytes read : ", rx_read);
      int i = 0;
      for (i = 0; i < rx_read; i++) {
        printf("%uc ", lidar_packet[i]);
      }
      printf("\n");
*/
   }

 }
 }
