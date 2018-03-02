/******************************************************************************
 * Header Declarations                                                    * 
 ******************************************************************************/
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

/******************************************************************************
 * Local Macro Declarations                                                    * 
 ******************************************************************************/
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define FPGA_PATH 		"/dev/spidev0.1" 
#define IRQ 6 // Use GPIO Pin 25, which is Pin 6 for wiringPi library
#define RST 21 // Use GPIO Pin 29, which is Pin 21 for wiringPi library
//#define SX_RST 21 // Use GPIO Pin 29, which is Pin 21 for wiringPi library
#define numSamples 2048
/******************************************************************************
 * Variable settings                                                 * 
 ******************************************************************************/
static uint32_t mode = 0;
static uint8_t bits = 8; // data is read as 8 bit usigned
static uint32_t speed = 3900000; //clock speed
static uint16_t delay_usecs = 10;
static int fpga_fd;
static uint8_t buf[4096] = {0,}; 
volatile int eventCounter = 0; // the event counter 
volatile uint8_t ready = 0;
uint8_t iq[numSamples+128] = {0,};
FILE *write_ptr;


/****************initializing the FPGA****************/
int fpga_init(char * fpga_path)
{
	// The following calls set up the path in FPGA
	if((fpga_fd = open(fpga_path, O_RDWR))<0){
		perror("SPI Error: Can't open device.");
		return -1;
	}
	if(ioctl(fpga_fd, SPI_IOC_WR_MODE, &mode)==-1){
		perror("SPI: Can't set SPI mode.");
		return -1;
	}
	if(ioctl(fpga_fd, SPI_IOC_RD_MODE, &mode)==-1){
		perror("SPI: Can't get SPI mode.");
		return -1;
	}
	if(ioctl(fpga_fd, SPI_IOC_WR_BITS_PER_WORD, &bits)==-1){
		perror("SPI: Can't set bits per word.");
		return -1;
	}
	if(ioctl(fpga_fd, SPI_IOC_RD_BITS_PER_WORD, &bits)==-1){
		perror("SPI: Can't get bits per word.");
		return -1;
	}
	if(ioctl(fpga_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't set max speed HZ");
		return -1;
	}
	if(ioctl(fpga_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}
	return 0;
}

/*********function to read in FPGA**********/
int fpga_read(uint8_t *data, uint16_t len)
{
	int ret;
	struct spi_ioc_transfer transfer = { //creating an spi transfer object which is read in
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay_usecs,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	
	transfer.len += len;
	
	ret = ioctl(fpga_fd, SPI_IOC_MESSAGE(1), &transfer);
	if(ret < 0)
		return ret;
	else
	{
		int j;
		for (j = 0; j < transfer.len; j++)
		{
			data[j] = buf[j];
		}
	}
	return ret;
}
/********function to generate an interrupt for data read********/
void myInterrupt(void) {
   	eventCounter++;
   	ready = 1;
   	fpga_read(iq, numSamples);
}
/****************************************************************/
int main(int argc, char* argv[]){

	// sets up the wiringPi library
	if (wiringPiSetup () < 0) {
		fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
		return 1;
	}

	//pinMode(SX_RST, OUTPUT);
	//digitalWrite(RST, HIGH);
	//usleep(10000);
	//digitalWrite(RST, LOW); 
	//usleep(5000);

	wiringPiSPISetup(0, 3900000);  //0 represents the channel and 3900000 Hz represents the clock speed

	
	/*************writing into registers*******************************/
	uint8_t cmd[2] = {0x90, 0x02};
	wiringPiSPIDataRW(0, cmd, 2); // SPI bus 0, cmd is buffer and 2 is the buffer length
	
	cmd[0] = 0x81; 
	cmd[1] = 0xC8;
	wiringPiSPIDataRW(0, cmd, 2);
	
	cmd[0] = 0x82;
	cmd[1] = 0x00;
	wiringPiSPIDataRW(0, cmd, 2);
	
	cmd[0] = 0x83;
	cmd[1] = 0x00;
	wiringPiSPIDataRW(0, cmd, 2);

	cmd[0] = 0x8C;
	cmd[1] = 0x3F;
	wiringPiSPIDataRW(0, cmd, 2);

	cmd[0] = 0x8D;
	cmd[1] = 0xF5;
	wiringPiSPIDataRW(0, cmd, 2);

	cmd[0] = 0x80;
	cmd[1] = 0x03;
	wiringPiSPIDataRW(0, cmd, 2);
	
	 /*******************************************************/
	speed = 15600000;
	delay_usecs = 1;
	fpga_init(FPGA_PATH); // initializing the FPGA by setting the path

	pinMode(RST, OUTPUT);
	digitalWrite(RST, HIGH);
	usleep(10000);
	digitalWrite(RST, LOW); 

  	// set Pin 25/0 generate an interrupt on high-to-low transitions
  	
	if ( wiringPiISR (IRQ, INT_EDGE_RISING, &myInterrupt) < 0 ) { // and attach myInterrupt() to the interrupt to indicate error
		fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
		return 1;
	}
	
	/******writing the generated values into a binary file********/
	write_ptr = fopen("samples.bin","wb");  // the data values are written into samples.bin, wb is for write in binary	
	int j = 0;
	while(j < 2000) // display counter value every second, 2000 is the number of samples
	{
		if(ready){
			ready = 0;
       		fwrite(iq,sizeof(iq[0]),numSamples,write_ptr);
        	j++;
		}
	}
	fclose(write_ptr);
	/*************************************************************/
	return 0;
}
