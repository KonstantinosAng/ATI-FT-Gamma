"""
Author: Konstantinos Angelopoulos
Date: 04/02/2020
All rights reserved.
Feel free to use and modify and if you like it give it a star.

Use on Linux or Windows only for Python 3.4.x+ (Tested for Python 3.6.8).
Simply read ascii or binary and translate to force (N) and torque (Nm).
"""
import serial
import time
import os


class Sensor(object):

    def __init__(self, port, mode='ascii'):
        """
        Initialize the connection to the ATI FT controller
        :param port: COM port that the sensor is connected on
        :param mode: communication mode (ascii or binary)
        :return None
        """
        print('[ATI FT SENSOR]: Connecting via serial...')
        try:
            # Initialize connection
            self.connection = serial.Serial(port, timeout=1, baudrate=38400, parity='N', stopbits=1)
            if os.name == 'nt':
                pass
            else:
                self.connection.open()  # Only for linux
            print('[ATI FT SENSOR]: Connected.')
            count = 0
            self._bias = [.0, .0, .0, .0, .0, .0]  # Store value to bias forces and torques
            self.reset_time = None
            self._mode = mode.lower()
            # Initialise Sensor
            self.initialise()
            while not self.start():
                count = count + 1
                self.initialise()
                if count == 20:
                    print('[ATI FT SENSOR]: Initialising... ')

            self.last_access = time.time()
            self.last_value = []
            print('[ATI FT SENSOR]: Started...')
        except Exception as e:
            print('[ATI FT SENSOR]: ATI FT Sensor not connected... \n {}'.format(e))

    def read(self):
        """
        The read string from the sensor is in format as bytes in binary mode:
        b'\x00\xff\x93\xff\x98\xfei\xff\xbb\x00\x06\xff\xdb\x06\r\n>QS'
        the first value \x00 is an error id that shows that everything is fine if its zero
        or that there is a problem if its not. The other 12 bytes must be paired to compute the counts,
        thus, \xff\x93 is the count for force in x axis \xff\x98 is the count in y axis and so on.
        These values are counts and must be divided properly according to the calibration (see counts_2_force function below).
        The /x06 is the <ACK> command, the /r is to match the carriage return and /n to make new line.
        :return bytes in format b'\x00\xff\x93\xff\x98\xfei\xff\xbb\x00\x06\xff\xdb\x06\r\n>QS'
        """
        if self._mode == 'binary':
            # read message in bytes
            read = self.connection.read(19)
            # Check if sensor has started
            while len(read) < 1 or read[0] != 0 or len(read) < 11 or read[-1] != 83:
                while not self.start():
                    print('[ATI FT SENSOR]: Fixing Sequence')
                read = self.connection.read(19)

            self.last_value = read
            self.last_access = time.time()
            return read
        else:
            """
            The read string from the sensor is in format as bytes in ascii form:
            b'0,  -102,   -99,  -225,   -20,    39,   -39\r\n'
            the first value 0 is an error id that shows that everything is fine if its zero
            or that there is a problem if its not. The other 6 ascii characters are the counts for each value,
            thus, -102 is the counts for force in x axis -99 is the counts in y axis and so on.
            These values are counts and must be divided properly according to the calibration (see counts_2_force function below). 
            The /r is to match the carriage return and /n to make new line.
            :return bytes in format b'0,  -102,   -99,  -225,   -20,    39,   -39'
            """
            read = self.connection.read(45)
            # while int(read.decode()[0]) != 0:
            while str(read)[2] != '0':
                while not self.start():
                    print('[ATI FT SENSOR]: Fixing Sequence')
                read = self.connection.read(45)
            return read[:-2].decode().split(',')

    def start(self):
        """
        Start Query string output reading from the sensor
        :return: Boolean
        """
        if self._mode == 'binary':
            self.connection.write(b'QS\r')
            msg_start = self.connection.read(5)
            # print(msg_start, len(msg_start))
            if len(msg_start) == 5 or str(msg_start)[2:4] == 'QS':
                return True
            else:
                return False
        else:
            self.connection.write(b'QS\r')
            msg_start = self.connection.read(5)
            if len(msg_start) == 5 and msg_start[0:2].decode() == 'QS':
                return True
            else:
                return False

    def stop(self):
        """
        Stop query output from sensor and close connection
        :return: bytes in format '\r\n\rn>'
        """
        self.connection.write(b'\r')
        self.connection.flushInput()
        self.connection.flush()
        msg_stop = self.connection.read(5)
        while msg_stop != b'\r\n\r\n>':
            self.connection.write(b'\r')
            self.connection.flushInput()
            self.connection.flush()
            msg_stop = self.connection.read(5)
        return msg_stop

    def initialise(self):
        """
        Reset connection and initialize mode before starting to pull data
        :return: None
        """
        current_time = time.time()
        if self.reset_time is not None:
            if current_time - self.reset_time < 2:
                self.reset_time = current_time
                return
        self.reset_time = current_time
        # print('FTSensor.reset(): Resetting at time', time.ctime())
        # Reset the sensor
        msg_stop = self.stop()
        # print(msg_stop)
        if self._mode == 'ascii':
            # Setup communication for binary output.
            self.connection.write(b'CD A\r')
            msg_reset = self.connection.read(11)
            # print(msg_reset)
            # Perform a moving average of 16 sensor data samples.
            self.connection.write(b'SA 16\r')
            msg_reset = self.connection.read(12)
            # print(msg_reset)
            # Sensor sampling Frequency allows optimizing for faster output when using CF.
            self.connection.write(b'SF\r')
            msg_reset = self.connection.read(15)
            # print(msg_reset)
            # Controls automatic SF optimization for RS-232 output.
            self.connection.write(b'CF\r')
            msg_reset = self.connection.read(31)
            # print(msg_reset)
            # Setup communication for Resolved force/torque data output (Default).
            self.connection.write(b'CD R\r')
            msg_reset = self.connection.read(11)
            # print(msg_reset)
            # Controls automatic SF optimization for RS-232 output.
            self.connection.write(b'CF 1\r')
            msg_reset = self.connection.read(11)
            # print(msg_reset)
            # Sensor sampling Frequency allows optimizing for faster output when using CF.
            self.connection.write(b'SF\r')
            msg_reset = self.connection.read(15)
            # print(msg_reset)
            # self.zero_bias()
            # Removes all previously stored biases from buffer.
            self.connection.write(b'SZ\r')
            msg_reset = self.connection.read(9)
            # print(msg_reset)
        else:
            # Setup communication for binary output.
            self.connection.write(b'CD B\r')
            msg_reset = self.connection.read(11)
            # print(msg_reset)
            # Perform a moving average of 16 sensor data samples.
            self.connection.write(b'SA 16\r')
            msg_reset = self.connection.read(12)
            # print(msg_reset)
            # Sensor sampling Frequency allows optimizing for faster output when using CF.
            self.connection.write(b'SF\r')
            msg_reset = self.connection.read(14)
            # print(msg_reset)
            # Controls automatic SF optimization for RS-232 output.
            self.connection.write(b'CF\r')
            msg_reset = self.connection.read(31)
            # print(msg_reset)
            # Setup communication for Resolved force/torque data output (Default).
            self.connection.write(b'CD R\r')
            msg_reset = self.connection.read(11)
            # print(msg_reset)
            # Controls automatic SF optimization for RS-232 output.
            self.connection.write(b'CF 1\r')
            msg_reset = self.connection.read(11)
            # print(msg_reset)
            # Sensor sampling Frequency allows optimizing for faster output when using CF.
            self.connection.write(b'SF\r')
            msg_reset = self.connection.read(13)
            # print(msg_reset)
            # self.zero_bias()
            # Removes all previously stored biases from buffer.
            self.connection.write(b'SZ\r')
            msg_reset = self.connection.read(9)
            # print(msg_reset)

    def zero_bias(self):
        """
        Performs a Sensor Bias. Stores bias reading in a 3-level buffer.
        ========== DONT USE, USE sensor_bias below
        :return None
        """
        # print('Sensor Biasing')
        self.connection.write(b'SB\r')
        msg_reset = self.connection.read(4)
        print(f'[ATI FT SENSOR]: {msg_reset}')

    def sensor_bias(self, _forces):
        """
        Performs a sensor force and torque bias
        :param _forces: list of forces in format [fx, fy, fz, tx, ty, tz] to subtract from measured values
        :return: None
        """
        self._bias = _forces

    def sensor_unbias(self):
        """
        Performs a sensor force and torque unbias
        :return: None
        """
        self._bias = [0, 0, 0, 0, 0, 0]

    def counts_2_force_torque(self, msg_binary, unbiased=False):
        """
        For different FT sensor counts see /ATI_FT/Calibration/ATI_FT_commands_manual.pdf
        For Nano25 and SI-125-3 Calibration Specifications
        counts force = 192
        counts torque = 10560
        Transform measured counts to forces and torques
        :param msg_binary: binary message in format b'\x00\xff\x93\xff\x98\xfei\xff\xbb\x00\x06\xff\xdb\x06\r\n>QS' pulled from the sensor
        :param unbiased: flag to return unbiased or biased force torque values
        :return list of forces and torques in format [fx, fy, fz, tx, ty, tz]
        """
        counts_force = 20.0  # for Gamma FT sensor and SI-65-5 Calibration Specifications
        counts_torque = 333.33  # for Gamma FT sensor and SI-65-5 Calibration Specifications
        if self._mode == 'binary':
            msg_binary = binary_2_counts(msg_binary)
            fx = msg_binary[0] + msg_binary[1]
            fy = msg_binary[2] + msg_binary[3]
            fz = msg_binary[4] + msg_binary[5]
            tx = msg_binary[6] + msg_binary[7]
            ty = msg_binary[8] + msg_binary[9]
            tz = msg_binary[10] + msg_binary[11]
            if unbiased:
                return [fx / counts_force, fy / counts_force, fz / counts_force, tx / counts_torque, ty / counts_torque, tz / counts_torque]
            return [-(fx / counts_force - self._bias[0]), -(fy / counts_force - self._bias[1]), fz / counts_force - self._bias[2], -(tx / counts_torque - self._bias[3]), -(ty / counts_torque - self._bias[4]), tz / counts_torque - self._bias[5]]
        else:
            fx = int(msg_binary[1])/counts_force
            fy = int(msg_binary[2])/counts_force
            fz = int(msg_binary[3])/counts_force
            tx = int(msg_binary[4])/counts_torque
            ty = int(msg_binary[5])/counts_torque
            tz = int(msg_binary[6])/counts_torque
            if unbiased:
                return [fx, fy, fz, tx, ty, tz]
            return [-(fx - self._bias[0]), -(fy - self._bias[1]), fz - self._bias[2], -(tx - self._bias[3]), -(ty - self._bias[4]), tz - self._bias[5]]


# Format to accommodate for extra bytes in message and exception
def binary_2_counts(binary_msg):
    """
    Convert to string
    _binary_msg = binary_msg[1:-4]
    Split message and check if there are more digits
    :param binary_msg: binary message in format b'\x00\xff\x93\xff\x98\xfei\xff\xbb\x00\x06\xff\xdb\x06\r\n>QS' pulled from the sensor
    :return list with forces and torques in format [fx, fy, fz, tx, ty, tz]
    """
    flag_check = str(binary_msg).split('\\')[2:]
    counter = 0
    if flag_check[-1] == "n>QS'":
        flag_check = flag_check[:-2]
    if len(flag_check) < 13:
        return binary_msg[1:13]
    check = [(len(x) > 3) for x in flag_check[:12]]
    # If no more byte found return
    if True not in check and len(binary_msg[1:]) >= 12:
        return binary_msg[1:13]
    for i in check:
        if i:
            counter += 1
    if len(flag_check) < 12 or len(flag_check) + counter < 12:
        return [binary_msg[1], binary_msg[2], binary_msg[3], binary_msg[4], binary_msg[5], binary_msg[6], binary_msg[7], binary_msg[8], binary_msg[9], binary_msg[10], binary_msg[11], binary_msg[12]]
    try:
        # If more digits were found calculate new message
        f, j = [], 0
        for i in range(1, len(binary_msg)):
            # account for the number of shifts to the right
            if len(f) > 11:
                break
            if check[i - 1]:
                f.append(binary_msg[i + j] + binary_msg[i + j + 1])
                # Shift to right by two
                j += 1
                continue
            f.append(binary_msg[i + j])
        return f
    except Exception:
        print(f'[ATI FT SENSOR]: {e}')
        # print(_binary_msg[0], _binary_msg[1], _binary_msg[2], _binary_msg[3], _binary_msg[4], _binary_msg[5], _binary_msg[6], _binary_msg[7], _binary_msg[8], _binary_msg[9], _binary_msg[10], _binary_msg[11])
        return [binary_msg[1], binary_msg[2], binary_msg[3], binary_msg[4], binary_msg[5], binary_msg[6], binary_msg[7], binary_msg[8], binary_msg[9], binary_msg[10], binary_msg[11], binary_msg[12]]


if __name__ == '__main__':
    """ Test functionality """
    from optparse import OptionParser
	import time
    parser = OptionParser()
    parser.add_option('--mode', action='store', default='run', type='string',
                      dest='mode', help='either "test" or "run"')

    (options, args) = parser.parse_args()

    if options.mode == 'test':
        a = b'\x00\xff\x9c\xff\x98\xff\x13\xff\xe7\x00\x1f\xff\xdd\x06\r\n>QS'
        forces = binary_2_counts(a)
        print(f"Fx: {'%.2f' % (forces[0]/40 + forces[1]/40)} N, Fy: {'%.2f' % (forces[2]/40 + forces[3]/40)} N, Fz: {'%.2f' % (forces[4]/40 + forces[5]/40)} N, "
              f"Tx: {'%.2f' % (forces[6]/333.33 + forces[7]/333.33)} Nm, Ty: {'%.2f' % (forces[8]/333.33 + forces[9]/333.33)} Nm, Tz: {'%.2f' % (forces[10]/333.33 + forces[11]/333.33)} Nm")
    else:
        daq = Sensor('COM1', mode='ascii')  # for linux probably /dev/ttyUSB0, use dmesg | grep tty to find the port
		start_time = time.time()
        while True:
            try:
                _msg = daq.read()
                forces = daq.counts_2_force_torque(_msg)

                print(f"Fx: {'%.3f' % (forces[0])} N, Fy: {'%.3f' % (forces[1])} N, Fz: {'%.3f' % (forces[2])} N, "
                      f"Tx: {'%.3f' % (forces[3])} Nm, Ty: {'%.3f' % (forces[4])} Nm, Tz: {'%.3f' % (forces[5])} Nm")

				# Bias Sensor
				if time.time() - start_time <= 5:
					forces = daq.counts_2_force_torque(_msg, unbiased=True)
					daq.sensor_bias(forces)
					print(f'Biased: {daq._bias}')

                # Restrict frequency (30 Hz)
                # time.sleep(1.0 / frequency - ((time.time() - start_time) % 1.0 / frequency))
            except Exception as e:
                print(e)
