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

// #define DEBUG																   // uncomment to enable DEBUG code
#define INTERVAL 60                                                            // default logging interval in seconds
#define TERMINAL    "/dev/ttyUSB0"											   // Usual device name for RS485 USB dongle

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

/*
 * Global variables
 */
char *progname;                                                                // holds our program name (argv[0]) globally

float voltage;                                                                 // values read from device
float current;
float power;
float energy;
float frequency;
float factor;
unsigned int alarmset;

/*
 * usage() prints passed error string, then usage string, and exits program
 */
void
usage (const char *errstring)
{
    fprintf (stderr,
             "%s\nUsage: %s [logging|newaddr] [<new address> (1-7)|<logging interval> (seconds)]\n",
             errstring, progname);
    exit (1);
}


/*
 * Open serial port and set up
 */
int openserial (char *portname)
{
    int fd;
    struct termios tty;

    fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        perror ("Error opening serial port");
        return -1;
    }

    if (tcgetattr (fd, &tty) < 0)											   // get attributes from port into termios struct
    {
        perror ("Error getting attributes from serial port");
        return -1;
    }

    cfsetospeed (&tty, (speed_t) B9600);									   // set baud rate for read and write
    cfsetispeed (&tty, (speed_t) B9600);


    tty.c_cflag |= (CLOCAL | CREAD );                                     	   // enable read, disable modem signals
    tty.c_cflag &= ~CSIZE;													   // Clear the size bits
    tty.c_cflag |= CS8;                                             		   // Now set just the 8 bit size bit
    tty.c_cflag &= ~PARENB;                                                    // no parity
    tty.c_cflag &= ~CSTOPB;                                                    // one stop bit
    tty.c_cflag &= ~CRTSCTS;                                                   // no hardware flow control

    /*
	 * Must use non-canonical mode with PZEM-16 device
	 * (character-at-a-time processing, rather than line based processing)
	 */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* timed read - wait for VTIME (tenths of) secs then return
     * regardless of number of chars read
     */
    tty.c_cc[VMIN] = 0;                                                        // don't wait for min chars read
    tty.c_cc[VTIME] = 5;                                                       // timeout and return after half a sec if no chars read

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        perror ("Error setting attributes on serial port");
        return -1;
    }
    return fd;
}


/*
 * set address of a new device, using one of the following command strings:
 */
int
setaddress (int fd, int newaddress)
{
/* 
 * Array of command strings to set particular addresses (needs specific CRC bytes for each command)
 */
	static
    unsigned char xstr[7][8] = { { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x01, 0xFD, 0xA3 }, // set address to 0x01
                                 { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x02, 0xBD, 0xA2 }, // set address to 0x02
                                 { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x03, 0x7C, 0x62 }, // set address to 0x03
                                 { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x04, 0x3D, 0xA0 }, // set address to 0x04
                                 { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x05, 0xFC, 0x60 }, // set address to 0x05
                                 { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x06, 0xBC, 0x61 }, // set address to 0x06
                                 { 0xF8, 0x06, 0x00, 0x02, 0x00, 0x07, 0x7D, 0xA1 } }; // set address to 0x07
    int xlen = 8;													           // command length will always be 8 chars
    unsigned char buf[8];													   // buffer for read chars back from device
    int count;																   // generic loop counter
    int wlen;														           // number of bytes actually written to device
    int rdlen;														           // number of bytes read back from device

	newaddress -=1;		    												   // newaddress indexes into array, so zero start

	printf("Writing:  ");
    for (count = 0; count < xlen; count++)
		printf("%X ", xstr[newaddress][count]);
	printf(" to device.\nExpecting the same bytes to be returned:\n");

    wlen = write (fd, xstr[newaddress], xlen);
    if (wlen != xlen)
    {
        fprintf (stderr, "Error: written %d chars, expected %d\n", wlen, xlen);
        return -1;
    }
    tcdrain (fd);                                                              // wait for all characters to write

    rdlen = read (fd, buf, 8);
#ifdef DEBUG
    printf("DEBUG: read %d chars back from device when setting new addr\n", rdlen);
#endif // DEBUG
    if (rdlen == 0)
    {
        fprintf (stderr, "No chars received after address change write\n");
        return -1;
    }

	printf("\nReceived: ");
    for (count = 0; count < rdlen; count++)
		printf("%X ", xstr[newaddress][count]);
	printf(" back from device.\n\n");
    return 0;
}

/*
 * Read the register values back from the device and convert to readable numbers
 * This function is called once the first 3 chars have been read from the device
 * This function should really put the result values into a structure,
 * but I'm lazy and just used separate variables
 */
int
readvalues (int fd)
{
    unsigned char buf[22];
    int rdlen;

    rdlen = read (fd, buf, 22);                                                // read remaining message from device

    if (rdlen < 22)
    {
        fprintf (stderr, "incorrect message length received: %d.\n", rdlen);
        return -1;
    }
    /* calculate parameter values from returned chars
     * and store in global variables
     */
    voltage = (float) ((buf[0] << 8) | buf[1]) / 10;
    current = (float) ((buf[4] << 24) | (buf[5] << 16) | (buf[2] << 8) | buf[3]) / 1000;
    power = (float) ((buf[8] << 24) | (buf[9] << 16) | (buf[6] << 8) | buf[7]) / 10000;
    energy = (float) ((buf[12] << 24) | (buf[13] << 16) | (buf[10] << 8) | buf[11]) / 1000;
    frequency = (float) ((buf[14] << 8) | buf[15]) / 10;
    factor = (float) ((buf[16] << 8) | buf[17]) / 100;
    alarmset = (buf[18] << 8) | buf[19];
    return 0;
}

/*
 * Send command and receive first part of response from power monitor
 * fd is opened serial port
 * Master (this machine) sends a command string to the power unit
 * and if it's correctly formatted (with correct CRC) the unit
 * responds with about 26 characters of message. 
 * This message is read into the serial port buffer automatically,
 * so when we read from the port, we're usually just reading from the
 * linux port buffer, which returns immediately.
 * This function reads the first 3 chars of the returned message,
 * to determine the correct response. The rest of the message stays in
 * the port buffer, to be read by the next function.
 *
 * NOTE last two digits of command string (xstr) are CRC in REVERSE ORDER
 * use site https://crccalc.com/ to calculate CRC (MODBUS 16bit format)
 * first char of message is device address. 0xF8 is "universal" address 
 * (all devices on bus will respond, so only use if a single device on bus)
 * Most purchased PZEM-16 devices are shipped with address 0x01
 */
int
sendcommand (int fd)
{
    // unsigned char xstr[]={0xF8, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x64, 0x64};
    unsigned char xstr[] = { 0x01, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x70, 0x0D };
    int xlen = 8;                                                              // length of command string to send (inc CRC)
    int wrlen;                                                                 // number of chars actually written to serial port
    int count;                                                                 // generic counter for loops
    int rdlen;                                                                 // number of chars actually read from serial port
    unsigned char buf[4];                                                      // temp storage for returned chars read from port

    /* send command to power unit */
    wrlen = write (fd, xstr, xlen);                                            // write command string to device 

    if (wrlen != xlen)                                                         // Not all chars were written, so write error
    {
        fprintf (stderr, "Error from write: %d, %d\n", wrlen, errno);
        return -1;
    }
    tcdrain (fd);                                                              /* delay for output */

    /* now read result from power unit */

    rdlen = read (fd, buf, 3);
    if (rdlen != 3)
    {
        fprintf (stderr, "error: read returned length %d\n", rdlen);
        for (count = 0; count < rdlen; count++)
        {
            fprintf (stderr, "char %d = %x\n", count, buf[count]);
        }
    }

    switch (buf[1])
    {
    case 0x84:
        fprintf (stderr, "message error code received. Error no. = %d\n",
                 buf[2]);
        return -1;
    case 0x04:
        return 0;
    default:
        fprintf (stderr, "unknown message type: %d.\n", buf[1]);
        return -1;
    }
}

/*
 * loop forever taking readings and logging them to stdout
 * Don't forget to fflush after every log entry, to flush the memory buffer
 * Create initial logfile on startup, then new logfiles every period (24 hrs?)
 * name logfiles with timestamp
 */
#define TIMEFILESTRING "pm-%Y-%m-%dT%H%M"                                      // time string in format suitable for filename (ISO 8601)
#define TIMELOGSTRING "%Y:%b:%d:%a:%X"                                         // time string in format suitable for log entry

void
logloop (int fd, int interval)
{
    char timestamp[26];                                                        // buffer to hold timestamp string
    FILE *logfile;                                                             // file pointer for log file
    pid_t pid;                                                                 // pid of process (parent or child)

    pid = fork ();                                                             // fork a new process to run in background
    if (pid < 0)                                                               // fork didn't work if returns less than 0
        usage ("Fork failed!");                                               // print error and exit program

    if (pid)                                                                   // pid > zero, so this is the parent
    {
        printf ("Logging process running in background. PID = %d\n\n", pid);   // in parent, fork returns PID of child
        exit (0);                                                              // child does the work now
    }
    /* if we get here, we should be child process */

    if (signal (SIGHUP, SIG_IGN) == SIG_ERR)                                   // ignore hangup so calling terminal can exit
        usage ("Failed to ignore SIGHUP signal");                             // print error and exit if we can't ignore SIGHUP

    time_t clk = time (NULL);                                                  // get current time
    strftime (timestamp, 26, TIMEFILESTRING, localtime (&clk));                // format timestamp suitable for filename and log entry
    logfile = fopen (timestamp, "w");                                          // create initial logfile when prog starts
    if (logfile == NULL)                                                       // create initial logfile when prog starts
        usage ("cannot open logfile");                                        // cannot open log file for some reason

    while (1)                                                                  // loop forever taking readings
    {
        time_t clk = time (NULL);                                              // get current time
        if (clk % 86400 == 0)                                                  // do this every so many seconds (eg 86400 = 1 day)
        {
            fclose (logfile);                                                  // every time period close log and start new file
            strftime (timestamp, 26, TIMEFILESTRING, localtime (&clk));        // format timestamp suitable for filename and log entry
            logfile = fopen (timestamp, "w");
            if (logfile == NULL)                                               // create initial logfile when prog starts
                usage ("cannot open logfile");                                // cannot open log file for some reason
        }
        strftime (timestamp, 26, TIMELOGSTRING, localtime (&clk));             // format timestamp suitable for filename and log entry
        sendcommand (fd);                                                      // send command and recieve initial ack from device
        readvalues (fd);                                                       // read rest of data from device and present
        fprintf (logfile, "%s,%.1f,%.2f,%.2f,%.2f,%.1f,%.2f\n",
                 timestamp, voltage, current, power, energy, frequency,
                 factor);
        fflush (logfile);                                                      // flush output to logfile
        sleep (interval);                                                      // repeat every specified number of seconds
    }
}





/*
 * most of the work is done by above functions
 * so main() just processes args and chooses feature to execute
 */
int
main (int argc, char *argv[])
{
    int fd;                                                                    // file descriptor for serial port
    char *portname = TERMINAL;                                                 // file name of serial port to open
    int newaddr;                                                               // new address argument

    progname = argv[0];                                                        // set my program name to a global, for use in usage()

    fd = openserial (portname);                                                // open serial port to device on MODBUS
    if (fd < 0)                                                                // can't do anything else 'til port is open
    {
        usage ("Error: cannot open serial port");
    }

    switch (argc)                                                              // test for arguments
    {
    case 1:                                                                   // no args, so just take a human readable reading and exit
        sendcommand (fd);                                                      // send command and recieve initial ack from device
        readvalues (fd);                                                       // read rest of data from device 
        printf                                                                 // print values to stdout
            ("voltage = %f\ncurrent = %f\npower = %f\nenergy = %f\nFrequency = %f\nfactor = %f\nalarm = %x\n",
                                                       voltage, current, power, energy, frequency, factor, alarmset);
        exit (0);

    case 2:                                                                   // only one argument, so should be "logging"
        if (strcmp (argv[1], "logging") == 0)                                  // strcmp returns zero if strings match
            logloop (fd, INTERVAL);                                            // Function logloop() loops forever so doesn't return
        else if (strcmp (argv[1], "newaddr") == 0)
            usage ("new address must be specified");
        else
            usage ("unknown argument");

    case 3:                                                                   // address change mode - needs two arguments 
        if (strcmp (argv[1], "newaddr") == 0)
        {
            newaddr = atoi (argv[2]);                                          // convert arg to int
            if ((newaddr > 0) && (newaddr <= 7))                               // second argument needs to be new address
            {
                setaddress(fd,atoi(argv[2]));
                exit (0);
            }                                                                  // if we get here, new address value out of range
            usage ("New address arg must be between 1 and 7");
        }                                                                      // if we get here, argument didn't match
        if (strcmp (argv[1], "logging") == 0)                                  // we've specified an argument to "logging" mode
            logloop (fd, atoi (argv[2]));                                      // run logging with specified argument

        usage ("Unknown argument");

    default:
        usage ("Too many arguments");
    }
    usage ("Should never get here");
}
