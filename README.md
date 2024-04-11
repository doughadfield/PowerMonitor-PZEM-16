# PowerMonitorPZEM
Read data from PZEM-016 power monitor module onto raspberry pi

new logfiles are created at 11:30pm (BST) or 10:30pm (GMT) every evening
Logfiles are created in current working directory, and named with timestamp

USAGE:
$ pm [options] [<serial port device>]

options:
    -l           = forks background process to log periodically to daily logfiles
    -r           = reset energy accumulator on device
    -a <address> = set address of device (command sent via universal address 0xF8)
    -d <address> = specify device address to operate on (default 0xF8
    -p <period>  = logging period in seconds (default 60 seconds)
    -h           = help (prints this message)

command with no arguments prints immediate readings from device
