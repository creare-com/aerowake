import smbus
import Adafruit_GPIO.I2C as I2C


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

    def read_byte_data(self, start_register, byte_count):
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
        
    def read_bit_data(self, start_register, byte_count):
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
        bits : string
            The data read from the sensor, as a string of bits.
            Byte ordering starts at start_register and moves upward through the memory space.
            Bit ordering is MSB to LSB, counting upward in list index.
            So if the device has 0x05 at register 0, and 0x07 at register 1,
            read_bit_data(0,2) will return '0000010100000111'.
        """
        return I2cSensor.parse_bits(self.read_byte_data(start_register, byte_count))
        
    @staticmethod
    def parse_bits(bytes):
        """
        Parameters
        ----------
        bytes : list of integers
            The input to break out into a string of bits, eg [5, 7].
            Each number must be less than 255.
            
        Returns
        -------
        bits : string
            A string of bytes representing the input, eg '0000010100000111'
        """
        bits = ''.join(['0'*(8-byte.bit_length())+ bin(byte)[2:] for byte in bytes])
        return bits

    def __del__(self):
        self._bus.close()
