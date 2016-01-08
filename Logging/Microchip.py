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
    
    def read_t(self):
        """
        Returns
        -------
        pressure : float
            The temperature, in degrees C, of the sensor die
        """
        bits = self.read_bit_data(0, 2)
        t = self.parse_t(bits)
        return t
    def parse_t(self, b):
        abs_t = int(b[4:], 2) * 2**-4 # Temperature is represented in 0.0625 degree C increments
        sign_bit = b[3] # 0 for temps >= 0C, 1 otherwise
        return abs_t if sign_bit == 0 else -abs_t
    def __del__(self):
        self._bus.close()
