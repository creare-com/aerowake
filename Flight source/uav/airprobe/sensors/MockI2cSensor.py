"""
Mock version of I2cSensor.py
"""

# import smbus
import struct
# import Adafruit_GPIO.I2C as I2C


class I2cSensor(object):
    def __init__(self, addr, desc='I2C Sensor'):
        """
        Parameters
        -----------
        addr : int
            i2c sensor address. Can be specified as (for example) 0x10
        desc : string
            Human-readable description to identify this sensor
        """
        self._address = addr
        self._description = desc
        
        # Open the bus
        busnum = I2C.get_default_bus()
        self._bus = smbus.SMBus(busnum)
    
    def get_desc(self):
        return self._description
    def set_desc(self, desc):
        self._description = desc

    def _read_byte_data(self, start_register, byte_count):
        """
        Read read a number of bytes starting at a particular register (address)
        
        Parameters
        -----------
        start_register : int
            The I2C register (or address) on the sensor from which to begin reading
        byte_count : int
            The number of bytes to read
            
        Returns
        -------
        data : list of integers
            The data read from the sensor, one byte per integer.
        """
        return self._bus.read_i2c_block_data(self._address, start_register, byte_count)
        
    @staticmethod
    def _two_bytes_to_one_word(two_bytes):
        """
        Parameters
        ----------
        two_bytes : list of exactly two bytes
            The input to combine, eg [5, 7].
            Each number must be less than 255.
            The first is considered the most significant byte.
            
        Returns
        -------
        one_word : integer
            An integer representing the input, eg 0x0507
        """

        return struct.unpack('H',struct.pack('BB',two_bytes[1],two_bytes[0]))[0]

    def __del__(self):
        self._bus.close()
