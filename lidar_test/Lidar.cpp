#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

#include <cstring>

#include "Lidar.h"
#include <cmath>
#include <inttypes.h>

#define PI 3.141592654

//-------------------------
//----- SETUP USART 0 -----
//-------------------------
//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
int uart0_filestream = -1;

/*Lidar Buffer Variables*/
//---------------------------------------------------------------------------------
lidar_packet lidar_packet_buf[LIDAR_BUF_SIZE];
int lidar_buf_in_ind;
int lidar_buf_out_ind;
int lidar_buf_count;

int packets_dropped = 0;
int bad_packets = 0;

int count = 0;
int correctPackageCount = 0;
int totalPackageCount = 1;
int data = 0;

//---------------------------------------------------------------------------------


/*Save Buffer Variables*/
//---------------------------------------------------------------------------------
bool storingData = false;

int buf_in_ind = 0;
int buf_out_ind = 0;
int bytes_to_read = 0;
uint8_t buf[BYTES_PER_PACKET * PACKETS_TO_SAVE];

int packets_ready = 0;
//---------------------------------------------------------------------------------


/*Saving Data Variables*/
//---------------------------------------------------------------------------------
const char data_filename[] = "/usr/matlab_data_LastDay_4.csv";
uint8_t empty_packet[] = {'i', ',', 'd','d', ',', 'd','d', ',', 'd','d', ',', 'd','d', '\n'};


int data_fd;
uint8_t *data_start_ptr;
uint8_t *data_cur_ptr;

bool save_lidar_data = false;

int bytes_per_update = PACKETS_PER_SECOND * BYTES_PER_PACKET / UPDATE_FREQ;
//---------------------------------------------------------------------------------

int abs(int i) {
    if (i > 0) return -1 * i;
    return i;
  }

void println(const char* s) {
  printf("%s\n", s);
}

void empty_buffer() {
   unsigned char b;
   unsigned char rx_buf[1];
   int rx_len = read(uart0_filestream, (void*)rx_buf, 1); // Read one char at a time
   while(rx_len > 0) {
     b = rx_buf[0];
     rx_len = read(uart0_filestream, (void*)rx_buf, 1); // Read one char at a time
     buf[buf_in_ind] = b;
     buf_in_ind++;
    
     bytes_to_read++;
     
     if(buf_in_ind >= BYTES_PER_PACKET * PACKETS_TO_SAVE) {
       buf_in_ind = 0;
     }
   }
 }
 
int flag_scan_complete = 0;
int scan_start_deg;
int scan_buf_ind;
uint8_t last_scan_id;
lidar_dist lidar_dist_buf[720];
lidar_dist lidar_dist_buf_complete[360];

void reset_scan_buf() {
	flag_scan_complete = 0;
	scan_buf_ind = 0;
	//Serial.println("new scan");
}

lidar_dist get_degree(uint16_t des_degree) {
	int des_ind;
	if(scan_start_deg < des_degree) {
      des_ind = des_degree - scan_start_deg;
    }
    else {
      des_ind = 360 - (scan_start_deg - des_degree);
    }
	return lidar_dist_buf_complete[des_ind];
}

int make_packet(int stop_i){
	uint8_t id;
	int bytes_read = 0;
	int dist_count = 0;
	uint8_t packet_buf[DATA_PACKET_LENGTH] = {'i', ',', 'd','d', ',', 'd','d', ',', 'd','d', ',', 'd','d', '\n'};
	char dist_buf[8];
	uint8_t temp[22];
  
	dist_count = 0;
  
	//Serial.println(buf_out_ind);
	while(buf_out_ind < stop_i && buf[buf_out_ind] != 0xFA) {
		buf_out_ind++; 
		bytes_to_read--;
	}
	if(buf_out_ind == stop_i) {
		//Serial.println("z");
		if(buf_out_ind == PACKETS_TO_SAVE * BYTES_PER_PACKET) {
			buf_out_ind = 0;
		}
		return 1;
	}
	//Serial.println(buf_out_ind);
	temp[0] = 0xFA;
  
	bytes_read++;
	buf_out_ind++;
	bytes_to_read--;
	if((temp[1] = buf[buf_out_ind]) < 0xA0 || temp[1] > 0xF9) {
		return 1;
	}
 
	bytes_read++;
	buf_out_ind++;
	bytes_to_read--;
 
	while(bytes_to_read > 0 && bytes_read < 22) {
		temp[bytes_read] = buf[buf_out_ind];
		bytes_read++;
		buf_out_ind++;
		bytes_to_read--;
		if(buf_out_ind == stop_i)
			buf_out_ind = 0;
	}
	if(bytes_read < 22) {
		return 1;
	}
  
	/* Correction */
	int k;
	int temp_ind;
  
	lidar_packet_buf[lidar_buf_in_ind].id = temp[1];
	for(k = 0; k < 4; k++) {
		temp_ind = 5+k*4;
		if(temp[temp_ind] & 0x80) {
			temp[temp_ind] = 0;
			temp[temp_ind-1] = 0;
		}
		lidar_packet_buf[lidar_buf_in_ind].dist[k] = (temp[temp_ind]  &0x3F) << 8;
		lidar_packet_buf[lidar_buf_in_ind].dist[k] |= temp[temp_ind-1];
	}  
	lidar_buf_in_ind = (lidar_buf_in_ind+1) % LIDAR_BUF_SIZE;
   
   // Write to file if we want to */
   if(save_lidar_data) {
		packet_buf[0] = temp[1];
		packet_buf[2] = temp[5] & 0x3F;
		packet_buf[3] = temp[4];
		packet_buf[5] = temp[9] & 0x3F;
		packet_buf[6] = temp[8];
		packet_buf[8] = temp[13] & 0x3F;
		packet_buf[9] = temp[12];
		packet_buf[11] = temp[17] & 0x3F;
		packet_buf[12] = temp[16];
		int j;
		for(j = 0; j < DATA_PACKET_LENGTH; j++) {
			*data_cur_ptr++ = packet_buf[j];
		}
		packets_ready++;
	}
	
	if(!flag_scan_complete) {
		uint8_t id = temp[1];
		//Serial.println(id);
		int diff;
		uint16_t start_deg = 4*(id - 160);
		lidar_dist cur_d;
		int i;
		if(scan_buf_ind == 0) {
			scan_start_deg = start_deg;
			/*Serial.print("d:");
			Serial.println(start_deg);*/
			last_scan_id = id; }
		else if(abs(diff = id - last_scan_id) > 1 && !(id == 160 && last_scan_id==250)) {
			if(diff > 0) {
				diff = 4*(diff-1); }
			else {
				diff = ((250 - last_scan_id) + (id - 160)-1) * 4;
			}
			/*Serial.print("diff:");
			Serial.print(diff);
			Serial.println();*/
			int last_deg_recorded = lidar_dist_buf[scan_buf_ind-1].degree+1;
			for(i = 0; i < diff && scan_buf_ind < 720; i++) {
				/*Serial.print("skip:");
				Serial.print((last_deg_recorded+i)%360);
				Serial.print('\t');
				Serial.print(i);
				Serial.print('\t');
				Serial.print(diff);
				Serial.print('\t');
				Serial.println(scan_buf_ind);*/
				lidar_dist_buf[scan_buf_ind].degree = (last_deg_recorded + i) % 360;
				lidar_dist_buf[scan_buf_ind].dist_x = 0;
				lidar_dist_buf[scan_buf_ind].dist_y = 0;
				if(scan_buf_ind >= 360) {
					lidar_dist_buf_complete[scan_buf_ind - 360] = lidar_dist_buf[scan_buf_ind];
				}
				scan_buf_ind++;
			}
		}
		last_scan_id = id;
		
		/* Read in degrees and distances */
		for(i = 0; i < 4 && scan_buf_ind < 720; i++) {
			uint16_t cur_deg = (start_deg + i) % 360;
			uint16_t cur_dist = ((temp[5 + 4*i] & 0x3F) << 8) | temp[4+4*i];
			
			
		/*Serial.print("ci:");
		Serial.print(cur_deg);
		Serial.print('\t');
		Serial.println(scan_buf_ind);*/
			/*if(cur_deg == 190) {
				if(scan_buf_ind < 360) {
					Serial.print("1:");
					Serial.print(cur_dist);
					Serial.print('\t');
					Serial.println(scan_buf_ind);
				}
				else {
					Serial.print("2:");
					Serial.print(cur_dist);
					Serial.print('\t');
					Serial.println(scan_buf_ind);
				}
			}*/
			
			lidar_dist_buf[scan_buf_ind].degree = cur_deg;
			lidar_dist_buf[scan_buf_ind].dist = cur_dist;
			lidar_dist_buf[scan_buf_ind].dist_x = (int16_t)(cur_dist * cos((double)PI/180 * (double)cur_deg));
			lidar_dist_buf[scan_buf_ind].dist_y = (int16_t)(cur_dist * sin((double)PI/180 * (double)cur_deg));
//			Serial.print(cur_dist);
//			Serial.print("\t");
//			Serial.println(sqrt(pow(lidar_dist_buf[scan_buf_ind].dist_x,2) + pow(lidar_dist_buf[scan_buf_ind].dist_y,2)));
			if(scan_buf_ind >= 360) {
				lidar_dist_buf_complete[scan_buf_ind - 360] = lidar_dist_buf[scan_buf_ind];
			}
			scan_buf_ind++;
		}
		
		/* Check if we're done */
		if(scan_buf_ind >= 719) {
			flag_scan_complete = 1;
			println("ey");
			int i;
			for(i = 0; i < 360; i++) {
				if(lidar_dist_buf_complete[i].dist_x == 0) {
					lidar_dist_buf_complete[i].dist = lidar_dist_buf[i].dist;
					lidar_dist_buf_complete[i].dist_x = lidar_dist_buf[i].dist_x;
					lidar_dist_buf_complete[i].dist_y = lidar_dist_buf[i].dist_y;
                                            
          printf ("%d: %" PRIu16 "", i, lidar_dist_buf_complete[i].degree);
          printf (" %" PRIu16 "", lidar_dist_buf_complete[i].dist);
          printf (" %" PRIu16 "", lidar_dist_buf_complete[i].dist_x);
          printf (" %" PRIu16 "", lidar_dist_buf_complete[i].dist_y);
          printf("\n");
          
				}
			}
      reset_scan_buf();
		}
	}
	
	lidar_buf_count++;
	return 0;
} 
 
void process_buffer() {
   int i;
   int end_i = buf_in_ind, wrap = 0;
   int res;
   if(buf_in_ind <= buf_out_ind) {
       end_i = BYTES_PER_PACKET * PACKETS_TO_SAVE;
       wrap = 1;
   }
   for(i = buf_out_ind; i < end_i && bytes_to_read >= BYTES_PER_PACKET; i++) {
     if((res = make_packet(end_i)) == 1) {
       packets_dropped++;
     }
     else if(res == 2) {
       bad_packets++;
     }
   }
   if(wrap) {
     buf_out_ind = 0;
     for(i = 0; i < buf_in_ind && bytes_to_read >= BYTES_PER_PACKET; i++) {
       if((res = make_packet(buf_in_ind)) == 1) {
         packets_dropped++;
       }
       else if(res == 2) {
         bad_packets++;
       }
     }
   }
 }

// To main
void init_lidar() {
	lidar_buf_count = 0;
	lidar_buf_in_ind = 0;
	lidar_buf_out_ind = 0;
	
	buf_in_ind = 0;
	buf_out_ind = 0;
}

// To main
const char data_filename_lidar[] = "/usr/LIDAR_";
const char data_filename_end_lidar[] = ".csv";
int init_lidar_data_save(int save_count) {

  //Serial.println('ey');
  char filename[100] = "";
  strcat(filename, data_filename_lidar);
  char str[15];
  sprintf(str, "%d", save_count);
  strcat(filename, str);
  strcat(filename, data_filename_end_lidar);
  println(filename);	
	
  packets_ready = 0;

  data_fd = open(filename, O_RDWR | O_CREAT | O_TRUNC);
 
  int res = lseek(data_fd, DATA_PACKET_LENGTH*DATA_NUM_LINES, SEEK_SET);
  if(res == -1) {
    println("no stretcheronis");
    close(data_fd);
    return 1;
  }
  
  res = write(data_fd, " ", 1);
  if(res == -1) {
    println("no writeronis");
    close(data_fd);
    return 1;
  }
  
  data_start_ptr = (uint8_t *)mmap(0, DATA_PACKET_LENGTH * DATA_NUM_LINES, PROT_WRITE | PROT_READ, MAP_SHARED, data_fd, 0);
  if(data_start_ptr == MAP_FAILED) {
    println("no maperonis");
    close(data_fd);
    return 1;
  }
  data_cur_ptr = data_start_ptr;
  
  save_lidar_data = true;
  
  return 0;
}

int init_lidar_data_save() {

  packets_ready = 0;

  data_fd = open(data_filename, O_RDWR | O_CREAT | O_TRUNC);
 
  int res = lseek(data_fd, DATA_PACKET_LENGTH*DATA_NUM_LINES, SEEK_SET);
  if(res == -1) {
    printf("no stretcheronis\n");
    close(data_fd);
    return 1;
  }
  
  res = write(data_fd, " ", 1);
  if(res == -1) {
    printf("no writeronis\n");
    close(data_fd);
    return 1;
  }
  
  data_start_ptr = (uint8_t *)mmap(0, DATA_PACKET_LENGTH * DATA_NUM_LINES, PROT_WRITE | PROT_READ, MAP_SHARED, data_fd, 0);
  if(data_start_ptr == MAP_FAILED) {
    printf("no maperonis\n");
    close(data_fd);
    return 1;
  }
  data_cur_ptr = data_start_ptr;
  
  save_lidar_data = true;
  
  return 0;
}

// To main
void loop_lidar() {

  //Serial.println("ey");
  //if(Serial1.available() >= 220) {
    empty_buffer();
  //}
  //Serial.println(bytes_to_read);
  
  if(bytes_to_read >= bytes_per_update) {
    process_buffer();
   /* Serial.println(packets_ready);
    Serial.println(bad_packets);
    Serial.println(packets_dropped);
    Serial.println(""); */
  }
  /*
  if(save_lidar_data && packets_ready + bytes_per_update >= DATA_NUM_LINES) {
    msync(data_start_ptr, packets_ready * DATA_PACKET_LENGTH, MS_SYNC);
    munmap(data_start_ptr, DATA_PACKET_LENGTH * DATA_NUM_LINES);
    close(data_fd);
    Serial.println("Errything clozed");
    save_lidar_data = false;
  }
  */
  
}

void stop_lidar_save_data() {
	msync(data_start_ptr, packets_ready * DATA_PACKET_LENGTH, MS_SYNC);
    munmap(data_start_ptr, DATA_PACKET_LENGTH * DATA_NUM_LINES);
    close(data_fd);
    printf("Errything clozed\n");
    save_lidar_data = false;
}

/* TODO: 
 * 1. add in code to change the name of the file that will be saved to
 * 2. compute checksums
 */
 
 int main(int ac, char** av) {
	
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
 
   init_lidar();
   init_lidar_data_save();
 
   while(true) {
       loop_lidar();
   }
 }