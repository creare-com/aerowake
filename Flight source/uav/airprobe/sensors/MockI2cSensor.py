"""
Stub version of I2cSensor.py
"""

# import smbus
import struct
# import Adafruit_GPIO.I2C as I2C


class I2cSensor(object):
    def __init__(self, addr, desc='I2C Sensor', busnum=None):
        """
        Parameters
        -----------
        addr : int
            i2c sensor address. Can be specified as (for example) 0x10
        desc : string
            Human-readable description to identify this sensor
        busnum : int or None 
            The number of the I2C device to connect to.  For example,
            to connect to /dev/i2c-0, put 0.  To connect to the default
            bus, put None.
        """
        self._address = addr
        self._description = desc
        
    def get_desc(self):
        return self._description
    def set_desc(self, desc):
        self._description = desc
