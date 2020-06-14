# ATI-FT-Gamma

# Description
Python files to read and pull data from the ATI FT Cotroller and the ATI Force Torque Gamma Sensor. Can be used with python 2.7.x and Python 3.6.x and on Windows and Linux.

# Installation

1. Connect the serial output from the ATI Contoller box to a Serial to USB adaptor and then plug in the USB cable to the computer.

2. The FTsensor.py (python2)/FTsensor3.py(python3) files have the Sensor class that can be imported as:
	```
	from FTsensor3 import Sensor
	daq = Sensor('COM1', mode='ascii')  # for linux probably /dev/ttyUSB0, use dmesg | grep tty to find the port
    	while True:
        	try:
            		msg = daq.read()
            		forces = daq.counts_2_force_torque(msg)
            			print("Fx: {} N, Fy: {} N, Fz: {} N, Tx: {} Nm, Ty: {} Nm, Tz: {} Nm".format('%.3f' % forces[0], '%.3f' % forces[1], '%.3f' % forces[2], '%.3f' % forces[3], '%.3f' % forces[4], '%.3f' % forces[5]))
           		 # Restrict frequency (30 Hz)
            		# time.sleep(1.0/30.0 - ((time.time() - start_time) % 1.0/30.0))
        	except Exception as e:
            		print(e)
	```
3. The Sensor class takes two arguments:
	- port: string with the connected port of the Cotroller (for Windows COM1 to COM6 and for linux /dev/ttyUSB0).
	For Linux use dmesg | grep tty to see where the ATI Controller is connected to. For Windows open the device manager and go to Port (COM & LPT) to find in which port the  ATI Controller is connected to.
	- mode: string with communication mode of the controller (use ascii the default value).
