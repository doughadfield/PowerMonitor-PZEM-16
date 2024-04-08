# PowerMonitorPZEM
Read data from PZEM-016 power monitor module onto raspberry pi

new logfiles are created every period (24 hours)
Logfiles are created in current working directory, and named with timestamp

USAGE:

$ pm (with no arguments)
    - prints immediate readings to stdout

$ pm logging
    - forks a background process to log to auto-generated logfiles
        one reading per minute; logfiles changed every 24 hrs

$ pm logging <period>
    - as above but set logging period (in seconds)

$ pm newaddr <address>
    - sets address of device to <address>
        range of addresses is 1-7
        NOTE: ensure only ONE device is on the RS485 bus when setting address

