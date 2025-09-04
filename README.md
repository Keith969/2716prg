# eprom_prg
A programmer for EPROMs. This version programs Intel 2716, TI 2516, 2732 and
2532 types.

This comes in 3 parts:
1) The PC (or Mac, Linux) program that reads/writes HEX format files
   This is a Qt based app using QSerialPort running in a thread to
   transfer data to/from the PIC using RS232.
2) The PIC program that receives (or transmits) HEX data.
   This receives cmds and data from (1), carries out the commands,
   and sends results back to (1). This is hardware specific.
3) The hardware that supports the programing.
   This is a PIC, the device to be programmed, and circuitry to provide
   a programming pulse to the device. This is EPROM specific.

   To compile the GUI:

1) You need a copy of the Qt GUI toolkit installed on your machine.
   See for example https://www.qt.io/download-open-source
   You also need a copy of Visual C++ 2022.
   Set the env var QTDIR to the location of your Qt installation.
   Windows users - open eprom_prg.sln (the solution file)
   Mac & Linux users - run qmake on the eprom_prj.pro file to generate a
   makefile.

2) With the executable built, you can then connect the PC to the hardware using
   a FTDI cable.  suitable one can be found on Amazon by searching for 
   "DTECH FTDI USB to TTL 3.3V Serial Adapter Debug Cable 0.1 inch Pitch 
   Female Socket Header UART IC FT232RL Chip Windows 10 8 7 Linux Mac (1.8m)"
   The USB end plugs into a USB A port, the 6 pin connector plugs into the
   header on the hardware, with the black wire going to the gnd pin.
   (Make sure this is correct, unfortunately the connector is reversible
   and getting it the wrong way could damage the PIC).

3) Use a jumper on the link for usb power to power the board via USB, else
   without the jumper you can apply 5v via the 'ext pwr' header.

4) If you need to program the PIC in situ you can connect the PicKit to the ICSP
   header on the hardware. The pin marked 1 goes to pin 1 of the PIC.

5) When initially powered, or after a reset, the hardware orange LDE will flash.
   At this stage you can run the eprom_prg app on your PC and select the device
   you want to program, set the baud rate (the default of 115,000 should be
   used unless you really want to change it), plus the serial interface which
   should be detected automatically. Then click on 'Init'. If successful the
   orange LED will stop flashing and the green LED will be lit, indicating the
   hardware is ready. The baud rate and device type will be echoed to the
   message area.

6) Now you can either READ the EPROM, CHECK if it's wiped clean ready for
   writing, or load a HEX file then use WRITE to write the hex data. VERIFY will
   read and check the EPROM contents against the loaded HEX file. You can save
   the hex file read from the EPROM to a file on the PC.

7) During writing a progress bar indicated how far you are writing the EPROM,
   also the orange LED will be lit and the green LED will flash periodically
   while writing.

8) If the red LED is lit there is a buffer overflow. Try erasing the EPROM,
   checking the serial link settings and try again.

Any issues, please email keith@peardrop.co.uk


