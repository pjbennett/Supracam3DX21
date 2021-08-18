# Supracam3DX21

ELX new system setup - April 2019
Using the Advantech PCI-1612 serial port card and Ubuntu

The Advantech PCI-1612 serial port card (EXAR 17V35X chip) requires an older version of Ubuntu in order to compile the Linux driver.  The latest, still available, version that will work with the Advantech driver is Ubuntu 14.04.6 LTS (Trusty Tahr).  However, the ELX software will not compile on this older version (not sure why, but gets error to use C99 and even using C99 running the program results in gtk widget errors).  Therefore the following process is used:

1. Install Ubuntu 14.04.6 LTS

2. Setup root password ($ sudo passwd root)

3. Download and install (compile and install) Advantech driver.  Make sure to run 'make' as root:  $ su root.  Run 'make' and then 'make install'

4. Verify driver loads (use dmesg | grep tty to see if serial ports were assigned)

5. Install all Ubuntu updates for 14.04 and reboot.

6. Upgrade Ubuntu to version 16.04.6 LTS (next version that is currently available)

7. Reinstall Advantech driver (from the previously compiled version) using "make install"

8. Install libgtk-3-dev ($ sudo apt-get install libgtk-3-dev)

9. Download elx software from GitHub

10. Use make to compile elx.c to elx

11. Verify elx runs properly (might need to change serial port names in elx-config.ini file)

12. Add user to dialout group (to access tty devices) using "sudo adduser <username> dialout". Need to logout and re-login to have access to the new group 




Notes:

If using Geany for development, use the following compile and build settings

compile:
gcc -Wall -c "%f" `pkg-config --cflags --libs gtk+-3.0` -rdynamic -pthread

build:
gcc -Wall -o "%e" "%f" `pkg-config --cflags --libs gtk+-3.0` -rdynamic -lm -pthread





Makefile:

SRC = elx.c
TARGET = elx

default: $(TARGET)

$(TARGET): elx.c
	gcc -Wall -o $(TARGET) $(SRC) `pkg-config --cflags --libs gtk+-3.0` -rdynamic -lm -pthread

clean:
rm -f $(TARGET)
