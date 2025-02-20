# 2708prg
A programmer for 2708 EPROMs.

This comes in 3 parts:
1) The PC (or Mac, Linux) program that reads/writes HEX format files
   This is a Qt based app using QSerialPort running in a thread to
   transfer data to/from the PIC using RS232.
2) The PIC program that receives (or transmits) HEX data.
   This receives cmds and data from (1), carries out the commands,
   and sends results back to (1).
3) The hardware that supports the programing.
   This is a PIC, the device to be programmed, and circuitry to provide
   a 25v programming pulse to the device.
