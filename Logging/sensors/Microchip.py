""" 
Contains classes to interface with sensors from the manufacturer called 'Microchip'.
(will not interface to every known microchip)
"""
from I2cSensor import I2cSensor 

class TemperatureSensor(I2cSensor):
    """
    Specifically the Microchip MCP9808 digital temperature sensor.
    """
    def __init__(self, addr, desc='Temperature Sensor'):
        """
        
        Parameters
        -----------
        addr : int
            i2c sensor address. Can be specified as (for example) 0x10
        desc : string
            Human-readable description to identify this sensor
        """
        super(TemperatureSensor, self).__init__(addr=addr, desc=desc)
    
    def read_temp_c(self):
        """
        Returns
        -------
        pressure : float
            The temperature, in degrees C, of the sensor die
        """
        bytes = self._read_byte_data(5, 2)
        t = self.parse_t(bytes)
        return t
        
    def read_temp_f(self):
        """
        Returns
        -------
        pressure : float
            The temperature, in degrees F, of the sensor die
        """
        return self.read_temp_c() * 1.8 + 32
        
    def parse_t(self, b):
        t_word = self._two_bytes_to_one_word(b) & 0x0FFF
        abs_t = t_word * 2**-4 # Temperature is represented in 0.0625 degree C increments
        sign_bit = (t_word & 0x1000) / 0x1000 # 0 for temps >= 0C, 1 otherwise
        return -abs_t if sign_bit else abs_t
    def __del__(self):
        self._bus.close()
