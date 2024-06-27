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

// #define DEBUG                                                               // uncomment to enable DEBUG code

#ifdef DEBUG
#define LOGINTERVAL 5                                                          // DEBUG logging interval in seconds
#define FILEINTERVAL 60                                                        // DEBUG log rotation interval
#define FILEINTOFFSET 0                                                        // DEBUG Offset in seconds after midnight for log change
#else
#define LOGINTERVAL 60                                                         // default logging interval in seconds
#define FILEINTERVAL 86400                                                     // Default log rotation interval (1 day)
#define FILEINTOFFSET 81000                                                    // Offset in seconds after midnight for log change
                                                                               // (81000 = 22 1/2 hours, to change at 11:30pm BST)
#endif                                                                         // DEBUG

#define TERMINAL    "/dev/ttyUSB0"                                             // Usual device name for RS485 USB dongle

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>								// needed for 32bit compile for older Pis
#include <time.h>
#include <signal.h>
#include <assert.h>

/*
 * Global variables
 */
char *progname;                                                                // holds our program name (argv[0]) globally

double voltage;                                                                // values read from device
double current;
double power;
double energy;
double frequency;
double factor;
unsigned int alarmset;
unsigned char buf[26];                                                         // read buffer for bytes read from serial port

/*
 * usage() prints passed error string, then usage string, and exits program
 */
void
usage (const char *errstring)
{
    if (errstring != NULL)
        fprintf (stderr, "%s\n", errstring);
    fprintf (stderr, "Usage: %s [options] [<serial port device>] (default /dev/ttyUSB0)\
\nOptions:\
\n    -l           = forks background process to log periodically to daily logfiles\
\n    -r           = reset energy accumulator on device\
\n    -a <address> = set address of device (command sent via universal address 0xF8 unless -d specified)\
\n    -d <device>  = specify device address to operate on (default 0xF8)\
\n    -p <period>  = logging period in seconds (default 60 seconds)\
\n    -h           = help (print this message)\n\n", progname);
    exit (1);
}


/*
 * Open serial port and set up
 */
int
openserial (char *portname)
{
    int fd;
    struct termios tty;

    fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        perror ("Error opening serial port");
        return -1;
    }

    if (tcgetattr (fd, &tty) < 0)                                              // get attributes from port into termios struct
    {
        perror ("Error getting attributes from serial port");
        return -1;
    }

    cfsetospeed (&tty, (speed_t) B9600);                                       // set baud rate for read and write
    cfsetispeed (&tty, (speed_t) B9600);


    tty.c_cflag |= (CLOCAL | CREAD);                                           // enable read, disable modem signals
    tty.c_cflag &= ~CSIZE;                                                     // Clear the size bits
    tty.c_cflag |= CS8;                                                        // Now set just the 8 bit size bit
    tty.c_cflag &= ~PARENB;                                                    // no parity
    tty.c_cflag &= ~CSTOPB;                                                    // one stop bit
    tty.c_cflag &= ~CRTSCTS;                                                   // no hardware flow control

    /*
     * Must use non-canonical mode with PZEM-16 device
     * (character-at-a-time processing, rather than line based processing)
     */
    tty.c_iflag &=
        ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
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
 * set address of a new device,
 * using the universal address (0xF8) to access the device
 * therefore only one device must be active on the bus 
 */
int
setaddress (int fd, int newaddress)
{
    /* 
     * Array of command strings to set particular addresses (needs specific CRC bytes for each command)
     */
    static unsigned char xstr[7][8] = { {0xF8, 0x06, 0x00, 0x02, 0x00, 0x01, 0xFD, 0xA3},   // set address to 0x01
                                        {0xF8, 0x06, 0x00, 0x02, 0x00, 0x02, 0xBD, 0xA2},   // set address to 0x02
                                        {0xF8, 0x06, 0x00, 0x02, 0x00, 0x03, 0x7C, 0x62},   // set address to 0x03
                                        {0xF8, 0x06, 0x00, 0x02, 0x00, 0x04, 0x3D, 0xA0},   // set address to 0x04
                                        {0xF8, 0x06, 0x00, 0x02, 0x00, 0x05, 0xFC, 0x60},   // set address to 0x05
                                        {0xF8, 0x06, 0x00, 0x02, 0x00, 0x06, 0xBC, 0x61},   // set address to 0x06
                                        {0xF8, 0x06, 0x00, 0x02, 0x00, 0x07, 0x7D, 0xA1} }; // set address to 0x07
    int xlen = 8;                                                              // command length will always be 8 chars
    int count;                                                                 // generic loop counter
    int wlen;                                                                  // number of bytes actually written to device
    int rdlen;                                                                 // number of bytes read back from device

    newaddress -= 1;                                                           // newaddress indexes into array, so zero start

#ifdef DEBUG
    printf ("Writing:  ");
    for (count = 0; count < xlen; count++)
        printf ("%X ", xstr[newaddress][count]);
    printf (" to device.\nExpecting the same bytes to be returned:\n");
#endif                                                                         // DEBUG
    wlen = write (fd, xstr[newaddress], xlen);
    if (wlen != xlen)
    {
        fprintf (stderr, "Error: write to device failed.\n");
        return -1;
    }
    tcdrain (fd);                                                              // wait for all characters to write

    rdlen = read (fd, buf, 8);
#ifdef DEBUG
    printf ("DEBUG: read %d chars back from device when setting new addr\n",
            rdlen);
    printf ("\nReceived: ");
    for (count = 0; count < rdlen; count++)
        printf ("%X ", xstr[newaddress][count]);
    printf (" back from device.\n\n");
#endif                                                                         // DEBUG
    if (rdlen == 0)
    {
        fprintf (stderr,
                 "Error: address change failed - no response from device.\n");
        return -1;
    }
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
    voltage = (double) ((buf[0] << 8) | buf[1]) / 10;
    current = (double) ((buf[4] << 24) | (buf[5] << 16) | (buf[2] << 8) | buf[3]) / 1000;
    power = (double) ((buf[8] << 24) | (buf[9] << 16) | (buf[6] << 8) | buf[7]) / 10000;
    energy = (double) ((buf[12] << 24) | (buf[13] << 16) | (buf[10] << 8) | buf[11]) / 1000;
    frequency = (double) ((buf[14] << 8) | buf[15]) / 10;
    factor = (double) ((buf[16] << 8) | buf[17]) / 100;
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
sendcommand (int fd, int device)
{
    unsigned char xstr[8][8] = { {0xF8, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x64, 0x64}, // send to universal address (only one device on bus)
                                 {0x01, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x70, 0x0D}, // send to device address 1 on bus
                                 {0x02, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x70, 0x3E},
                                 {0x03, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x71, 0xEF},
                                 {0x04, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x70, 0x58},
                                 {0x05, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x71, 0x89},
                                 {0x06, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x71, 0xBA},
                                 {0x07, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x70, 0x6B} };
    int xlen = 8;                                                              // length of command string to send (inc CRC)
    int wrlen;                                                                 // number of chars actually written to serial port
    int count;                                                                 // generic counter for loops
    int rdlen;                                                                 // number of chars actually read from serial port

    /* send command to power unit */
    wrlen = write (fd, xstr[device], xlen);                                    // write command string to device 

    if (wrlen != xlen)                                                         // Not all chars were written, so write error
    {
        fprintf (stderr, "Error from write: %d, %d\n", wrlen, errno);
        return -1;
    }
    tcdrain (fd);                                                              // wait for command to fully transmit

    /* now read result from power unit */

    rdlen = read (fd, buf, 3);
    if (rdlen != 3)
    {
        fprintf (stderr, "error: read from device returned %d bytes.\n",
                 rdlen);
        for (count = 0; count < rdlen; count++)
        {
            fprintf (stderr, "char %d = %x\n", count, buf[count]);
        }
        exit (-1);
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
* alarm handler does nothing
* it's only here to break out of sleep in the log loop
*/
void
alarm_handler (int sig)
{
#ifdef DEBUG
    printf ("In alarm handler - sig = %d.\n", sig);
#endif                                                                         // DEBUG
}

/*
 * initialise signal timer alarm
 */
void
timer_init (void)
{
    static sigset_t block;                                                     // structure to hold signal set

    sigemptyset (&block);                                                      // initialise and clear signal block
    sigaddset (&block, SIGALRM);                                               // add just SIGALRM signal to our block

    struct sigaction act = { 0 };                                              // structure for sigaction() call
    struct timeval interval;                                                   // structures for setitimer() call
    struct itimerval period;

    act.sa_handler = alarm_handler;
    assert (sigaction (SIGALRM, &act, NULL) == 0);                             // sigaction() manages catching SIGALRM signal

    interval.tv_sec = 1;                                                       // alarm every second to wake up sleep in log loop
    interval.tv_usec = 0;                                                      // zero microseconds
    period.it_interval = interval;
    period.it_value = interval;
    setitimer (ITIMER_REAL, &period, NULL);                                    // setitimer() starts countdown timer for SIGALRM
}



/*
 * reset_energy() will send the command to the device to reset the 
 * cumulative energy counter.
 * We reset the energy counter at every logfile switch
 * as well as a command line arg to reset manually
 * parameters are file descriptor (fd) and device address
 */
int
reset_energy (int fd, int device)
{
    unsigned char xstr[8][4] = { {0xF8, 0x42, 0xC2, 0x41},                     // command to reset energy counter
                                 {0x01, 0x42, 0x80, 0x11},                     // array of commands for device addresses 1-7
                                 {0x02, 0x42, 0x80, 0xE1},                     // and universal address
                                 {0x03, 0x42, 0x81, 0x71},
                                 {0x04, 0x42, 0x83, 0x41},
                                 {0x05, 0x42, 0x82, 0xD1},
                                 {0x06, 0x42, 0x82, 0x21},
                                 {0x07, 0x42, 0x83, 0xB1}
    };
    int xlen = 4;                                                              // command length will always be 4 chars
    int count;                                                                 // generic loop counter
    int wlen;                                                                  // number of bytes actually written to device
    int rdlen;                                                                 // number of bytes read back from device

#ifdef DEBUG
    printf ("Writing:  ");
    for (count = 0; count < xlen; count++)
        printf ("%X ", xstr[device][count]);
    printf (" to device.\nExpecting the same bytes to be returned:\n");
#endif                                                                         // DEBUG

    wlen = write (fd, xstr[device], xlen);
    if (wlen != xlen)
    {
        fprintf (stderr, "Error: written %d chars, expected %d\n", wlen,
                 xlen);
        return -1;
    }
    tcdrain (fd);                                                              // wait for all characters to write

    rdlen = read (fd, buf, 4);
#ifdef DEBUG
    printf ("DEBUG: read %d chars back from device when resetting energy\n",
            rdlen);
    printf ("\nReceived: ");
    for (count = 0; count < rdlen; count++)
        printf ("%X ", xstr[device][count]);
    printf (" back from device.\n\n");
#endif                                                                         // DEBUG

    if (rdlen == 0)
    {
        fprintf (stderr,
                 "error: no chars received when resetting energy counter.\n");
        return -1;
    }
    return 0;
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
logloop (int fd, int device, int interval)
{
    char timestamp[26];                                                        // buffer to hold timestamp string
    FILE *logfile;                                                             // file pointer for log file
    pid_t pid;                                                                 // pid of process (parent or child)

    pid = fork ();                                                             // fork a new process to run in background
    if (pid < 0)                                                               // fork didn't work if returns less than 0
        usage ("Fork failed!");                                                // print error and exit program

    if (pid)                                                                   // pid > zero, so this is the parent
    {
        printf ("Logging process running in background. PID = %d\n\n", pid);   // in parent, fork returns PID of child
        exit (0);                                                              // child does the work now
    }
    /* if we get here, we should be child process */

    if (signal (SIGHUP, SIG_IGN) == SIG_ERR)                                   // ignore hangup so calling terminal can exit
        usage ("Failed to ignore SIGHUP signal");                              // print error and exit if we can't ignore SIGHUP

    time_t clk = time (NULL);                                                  // get current time
    strftime (timestamp, 26, TIMEFILESTRING, localtime (&clk));                // format timestamp suitable for filename and log entry
    logfile = fopen (timestamp, "w");                                          // create initial logfile when prog starts
    if (logfile == NULL)                                                       // create initial logfile when prog starts
        usage ("cannot open logfile");                                         // cannot open log file for some reason

    timer_init ();                                                             // Start SIGALRM timer interrupts every second

    while (1)                                                                  // loop forever taking readings
    {
        time_t clk = time (NULL);                                              // get current time
        if ((clk % FILEINTERVAL) == FILEINTOFFSET)                             // do this every day (modulo 86400 secs) with offset for time of day
        {
#ifdef DEBUG
            printf ("Switching Logfile\n");
#endif                                                                         // DEBUG
            fclose (logfile);                                                  // every time period close log and start new file
            strftime (timestamp, 26, TIMEFILESTRING, localtime (&clk));        // format timestamp suitable for filename and log entry
            logfile = fopen (timestamp, "w");
            if (logfile == NULL)                                               // create initial logfile when prog starts
                usage ("cannot open logfile");                                 // cannot open log file for some reason
            reset_energy (fd, 1);                                              // reset the energy accumulation counter on device
        }

        if ((clk % interval) == 0)                                             // we've reached an "inverval" of seconds
        {
            strftime (timestamp, 26, TIMELOGSTRING, localtime (&clk));         // format timestamp suitable for filename and log entry
#ifdef DEBUG
            printf ("DEBUG: %s\n", timestamp);
            fprintf (logfile, "DEBUG: %s\n", timestamp);
#else
            sendcommand (fd, device);                                          // send command and recieve initial ack from device
            readvalues (fd);                                                   // read rest of data from device and present
            fprintf (logfile, "%s,%05.1f,%04.1f,%04.1f,%05.1f,%.1f,%.2f\n",
                     timestamp, voltage, current, power, energy, frequency,
                     factor );
#endif                                                                         // DEBUG
            fflush (logfile);                                                  // flush output to logfile
        }
        sleep (100);                                                           // Sleep longer than interrupt timer interval
    }
}


/*
 * main routine
 * most of the work is done by above functions
 * so main() just processes args and chooses feature to execute
 */
int
main (int argc, char *argv[])
{
    int fd;                                                                    // file descriptor for serial port
    char *portname = TERMINAL;                                                 // file name of serial port to open
    int newaddr;                                                               // new address argument
    int period = LOGINTERVAL;
    int device = 0;                                                            // default device address
    int address = 0;                                                           // initialise to error value 'till set by specific option
    int resetflag = 0;
    int logflag = 0;
    int c;                                                                     // char returned from getops()
    int cmd;                                                                   // return value from sendcommand()

    progname = argv[0];                                                        // set my program name to a global, for use in usage()

    /*
     * Parse command line options (see usage() function above for arg descriptions)
     */
    while ((c = getopt (argc, argv, "lra:d:p:h")) != -1)
    {
        switch (c)
        {
        case 'p':
            period = atoi (optarg);
            if (period < 1 || period > 3600)
                usage ("Period must be between 1 and 3600");
            break;
        case 'd':
            device = atoi (optarg);
            if (device < 0 || device > 7)
                usage ("Device address must be between 0 and 7");
            break;
        case 'a':
            address = atoi (optarg);
            if (address < 1 || address > 7)
                usage ("New address must be between 1 and 7");
            break;
        case 'r':
            resetflag++;                                                       // perform actual reset outside getopts loop, to ensure we get all opts
            break;
        case 'l':
            logflag++;                                                         // call logging routine outside getopts loop, to ensure we get all opts
            break;
        case 'h':
            usage ("");                                                        // usage() call exits program, so doesn't return here
            break;
        case '?':                                                             // error return from getopts
            usage (NULL);
        default:
            usage
                ("At default case in getopt switch - should never get here");
        }
    }
    /*
     * we've exhausted the options, but there may be one more argument (USB serial device)
     */
    if (argc > (optind + 1))                                                   // there are outstanding arguments
        usage ("Too many arguments");                                          // only one additional arg supported
    if (argc > optind)                                                         // we have one additional argument
        portname = argv[optind];                                               // argument is new serial port name

    fd = openserial (portname);                                                // open serial port to device on MODBUS
    if (fd < 0)                                                                // can't do anything else 'til port is open
    {
        usage ("Error: cannot open serial port");
    }

    if (address)                                                               // address initialised to zero, so has been set
    {
        setaddress (fd, address);
        device = address;                                                      // from now on use the new address for any other commands
    }

    if (resetflag)
    {
        printf ("About to reset energy accumulator on device %d\n", device);
        if (reset_energy (fd, device) == 0)
            printf ("Energy counter reset success\n");
        else
            printf ("Energy counter reset FAILED\n");
    }

    if (logflag)                                                               // logging option set
        logloop (fd, device, period);                                          // logloop doesn't return

    /*
     * if we get here there are no outstanding opts or args,
     * so just print immediate values from device and exit
     */
    cmd = sendcommand (fd, device);                                            // send command and recieve initial ack from device
    if (cmd < 0)
    {
        read (fd, buf, 20);                                                    // on error in sendcommand, flush port before exit
        exit (1);                                                              // on error in sendcommand, run readvalues() before exit, to flush port
    }
    readvalues (fd);                                                           // read rest of data from device 
    printf                                                                     // print values to stdout, right aligned
        ("Voltage:%8.2f\nCurrent:%8.2f\nPower:%10.2f\nEnergy:%9.2f\nFrequency:%6.2f\nfactor:%9.2f\nalarm:%7x\n",
		                                            voltage, current, power, energy, frequency, factor, alarmset);
    exit (0);
}
