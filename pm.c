/*
 * Copyright: Doug Hadfield 2024
 * Program to read data from PZEM-16 power monitor device
 * and either print to screen or log to stdout
 * thanks to the contributor to stackoverflow thread below:
 * https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
 * for the sample serial port I/O code.
 *
 * Writing to the PZEM-16 requires the CRC bytes at the end of the message to be correct.
 * The format is CRC16 MODBUS, but the words are REVERSED (LSW first, MSW second).
 * use site https://crccalc.com/ to calculate CRC (MODBUS 16bit format)
 */

#define INTERVAL 60
#define TERMINAL    "/dev/ttyUSB0"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <stdbool.h>

/*
 * Global variables
 */
char *progname;					// holds our program name (argv[0]) globally
bool logging = false;			// flag to indicate whether we're logging mode

int set_interface_attribs(int fd, int speed)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0)
	{
		fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t) speed);
	cfsetispeed(&tty, (speed_t) speed);

	tty.c_cflag |= (CLOCAL | CREAD);												/* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;																/* 8-bit characters */
	tty.c_cflag &= ~PARENB;															/* no parity bit */
	tty.c_cflag &= ~CSTOPB;															/* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;														/* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &=
		~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL |
		  IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

/*
 * Set VMIN and VTIME on the serial port
 * to control read timeout etc.
 * explanation at http://unixwiz.net/techtips/termios-vmin-vtime.html
 */
void set_mincount(int fd, int mcount)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0)
	{
		fprintf(stderr, "Error tcgetattr: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;												// 0 = timed read - timeout if no chars received
	tty.c_cc[VTIME] = 5;															// number of tenths of seconds to wait

	if (tcsetattr(fd, TCSANOW, &tty) < 0)
		fprintf(stderr, "Error tcsetattr: %s\n", strerror(errno));
}

/*
 * Open serial port and set up
 */
int openserial(char *portname)
{
	int fd;

	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		fprintf(stderr, "Error opening %s: %s\n", portname,
				strerror(errno));
		return -1;
	}

	/*baudrate B9600, 8 bits, no parity, 1 stop bit */
	set_interface_attribs(fd, B9600);
	set_mincount(fd, 0);															/* set to pure timed read */
	return fd;
}

/*
 * Read the register values back from the device and convert to readable numbers
 * This function is called once the first 3 chars have been read from the device
 * This function should really put the result values into a structure,
 * but I'm lazy and just used separate variables
 */
int displayvalues(int fd)
{
	unsigned char buf[22];
	int rdlen;
	unsigned int voltage;
	unsigned int current;
	unsigned int power;
	unsigned int energy;
	unsigned int frequency;
	unsigned int factor;
	unsigned int alarm;

	rdlen = read(fd, buf, 22);

#ifdef DEBUG
	fprintf(stderr, "read %d characters from serial port\n", rdlen);
#endif							// DEBUG

	if (rdlen < 22)
	{
		fprintf(stderr, "incorrect message length received: %d.\n", rdlen);
		return -1;
	}
	/* print values received from power monitor */
	voltage = (buf[0] << 8) | buf[1];
	current = (buf[4] << 24) | (buf[5] << 16) | (buf[2] << 8) | buf[3];
	power = (buf[8] << 24) | (buf[9] << 16) | (buf[6] << 8) | buf[7];
	energy = (buf[12] << 24) | (buf[13] << 16) | (buf[10] << 8) | buf[11];
	frequency = (buf[14] << 8) | buf[15];
	factor = (buf[16] << 8) | buf[17];
	alarm = (buf[18] << 8) | buf[19];

	if (logging)
	{
		char timestamp[26];
		time_t clk = time(NULL);	// get current time
		// ctime_r(&clk, timestamp);													// fill in "timestamp" buffer with human readable time stamp
        strftime(timestamp, 26, "%Y:%b:%d:%a:%X", localtime(&clk));
		// timestamp[24] = 0;															// delete the trailing newline from the timestamp string
		printf("%s,%.1f,%.2f,%.2f,%.2f,%.1f,%.2f\n", timestamp,
			   (float) voltage / 10, (float) current / 1000,
			   (float) power / 10000, (float) energy / 1000,
			   (float) frequency / 10, (float) factor / 100);
	} else
	{
		printf
			("voltage = %f\ncurrent = %f\npower = %f\nenergy = %f\nFrequency = %f\nfactor = %f\nalarm = %d\n",
			 (float) voltage / 10, (float) current / 1000,
			 (float) power / 10000, (float) energy / 1000,
			 (float) frequency / 10, (float) factor / 100, alarm);
	}
	return 0;
}

/*
 * set address of a new device, using one of the following command strings:
 */
int setaddress(int fd, int newaddress)
{
	unsigned char xstr1[] = { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x01, 0xFD, 0xA3 };		// set address to 0x01
	unsigned char xstr2[] = { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x02, 0xBD, 0xA2 };		// set address to 0x02
	unsigned char xstr3[] = { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x03, 0x7C, 0x62 };		// set address to 0x03
	unsigned char xstr4[] = { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x04, 0x3D, 0xA0 };		// set address to 0x04
	unsigned char xstr5[] = { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x05, 0xFC, 0x60 };		// set address to 0x05
	unsigned char xstr6[] = { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x06, 0xBC, 0x61 };		// set address to 0x06
	unsigned char xstr7[] = { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x07, 0x7D, 0xA1 };		// set address to 0x07
	unsigned char *xstr;
	int xlen = 8;
	int buf[8];
	int count;
	int wlen;
	int rdlen;

	switch (newaddress)
	{
	case 0x01:
		xstr = xstr1;
		break;

	case 0x02:
		xstr = xstr2;
		break;

	case 0x03:
		xstr = xstr3;
		break;

	case 0x04:
		xstr = xstr4;
		break;

	case 0x05:
		xstr = xstr5;
		break;

	case 0x06:
		xstr = xstr6;
		break;

	case 0x07:
		xstr = xstr7;
		break;
	}

	wlen = write(fd, xstr, xlen);
	if (wlen != xlen)
	{
		fprintf(stderr, "Error: written %d chars, expected %d\n", wlen,
				xlen);
		return -1;
	}
	tcdrain(fd);																	/* delay for output */

	rdlen = read(fd, buf, 8);
	if (rdlen = 0)
	{
		fprintf(stderr, "No chars received after address change write\n");
		return -1;
	}

	for (count = 0; count < rdlen; count++)
		printf("recieved char %d = %x\n", count, buf[count]);

	return 0;
}

/*
 * take reading from power monitor
 * fd is opened serial port
 * buf is address of result buffer (needs to be 26 chars min)
 */
int takereading(int fd)
{
/*
 * command string to sent to device. NOTE last two digits are CRC in REVERSE ORDER
 * use site https://crccalc.com/ to calculate CRC (MODBUS 16bit format)
 */
	// unsigned char xstr[]={0xF8, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x64, 0x64};
	unsigned char xstr[] =
		{ 0x01, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x70, 0x0D };
	int xlen = 8;				// length of above command string
	int wrlen;					// number of chars actually written to serial port
	int count;					// generic counter for loops
	int rdlen;					// number of chars actually read from serial port
	unsigned char buf[4];		// temp storage for returned chars read from port

	/* send command to power unit */
	wrlen = write(fd, xstr, xlen);

#ifdef DEBUG
	printf("Written %d chars\n", wrlen);											//DEBUG
#endif							// DEBUG

	if (wrlen != xlen)
	{
		fprintf(stderr, "Error from write: %d, %d\n", wrlen, errno);
		return -1;
	}
	tcdrain(fd);																	/* delay for output */

	/* now read result from power unit */

	rdlen = read(fd, buf, 3);
	if (rdlen != 3)
	{
		fprintf(stderr, "error: read returned length %d\n", rdlen);
		for (count = 0; count < rdlen; count++)
		{
			fprintf(stderr, "char %d = %x\n", count, buf[count]);
		}
	}

	switch (buf[1])
	{
	case 0x84:
		fprintf(stderr, "message error code received. Error no. = %d\n",
				buf[2]);
		return -1;
	case 0x04:
		displayvalues(fd);
		return 0;
	default:
		fprintf(stderr, "unknown message type: %d.\n", buf[1]);
		return -1;
	}
}

/*
 * loop forever taking readings and logging them to stdout
 * Don't forget to fflush after every log entry, to flush the memory buffer
 */
void logloop(int fd)
{
	logging = true;																	// set logging flag, to print in log format
	while (1)																		// loop forever taking readings
	{
		takereading(fd);															// get reading from device
		fflush(stdout);																// flush output to logfile
		sleep(INTERVAL);															// repeat every INTERVAL seconds
	}
}

/*
 * perror() prints passed error string, then usage string, and exits program
 */

void perror(const char *errstring)
{
	fprintf(stderr,
			"%s\nUsage: %s [logging|newaddr] [<new address> (1-7)]\n",
			errstring, progname);
	exit(1);
}



/*
 * most of the work is done by above functions
 * so main() just processes args and chooses feature to execute
 */

int main(int argc, char *argv[])
{
	int fd;						// file descriptor for serial port
	char *portname = TERMINAL;	// file name of serial port to open
	int newaddr;				// new address argument

	progname = argv[0];																// set my program name to a global, for use in perror()

	fd = openserial(portname);														// open serial port to device on MODBUS
	if (fd < 0)																		// can't do anything else 'til port is open
	{
		perror("Error: cannot open serial port");
	}

	switch (argc)																	// test for arguments
	{
	case 1:																	    	// no args, so just take a human readable reading and exit
		takereading(fd);
		exit(0);

	case 2:																	    	// only one argument, so should be "logging"
		if (strcmp(argv[1], "logging") == 0)										// strcmp returns zero if strings match
			logloop(fd);															// Function logloop() loops forever so doesn't return
        else if (strcmp(argv[1], "newaddr") == 0)
                perror ("new address must be specified");
            else
                perror("unknown argument");

	case 3:																	    	// address change mode - needs two arguments 
		if (strcmp(argv[1], "newaddr") == 0)
		{
			newaddr = atoi(argv[2]);												// convert arg to int
			if ((newaddr > 0) && (newaddr <= 7))									// second argument needs to be new address
			{
				// setaddress(fd,stoa(argv[2]);
				printf("About to call setaddress() with address %d\n", newaddr);
				exit(0);
			}																		// if we get here, new address value out of range
			perror("New address arg must be between 1 and 7");
		}																			// if we get here, argument didn't match
		perror("Unknown argument");

	default:
		perror("Too many arguments");
	}
	perror("Should never get here");
}
