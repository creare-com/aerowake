""" 

Contains classes to interface with sensors from the manufacturer called 'All Sensors'.
(Does not actually claim to contain all sensors)
"""
from I2cSensor import I2cSensor 


class PressureSensor(I2cSensor):
    """
    This class handles math and interactions common among All Sensors pressure sensors.
    """
    def __init__(self, addr, FSS=0.5, OSdig=1638, desc='Pressure Sensor'):
        """
        
        Parameters
        -----------
        addr : int
            i2c sensor address. Can be specified as (for example) 0x10
        FSS : float
            Sensor full scale level span in the same units as the sensor output (either inH20 or psi)
        OSdig : int
            Digital offset of sensor word
        desc : string
            Human-readable description to identify this sensor
        """
        super(PressureSensor, self).__init__(addr=addr, desc=desc)
        # Save attributes
        self.FSS = FSS
        self.OSdig = OSdig
    
    def read_p(self):
        """
        Returns
        -------
        pressure : float
            The pressure indicated by the sensor, in the same units that FSS was provided to __init__ earlier
        """
        bits = self.read_bit_data(0, 2)
        p = self.parse_p(bits)
        return p
    def read_pt(self):
        bits = self.read_bit_data(0, 4)
        return self.parse_pt(bits)
    def parse_pt(self, b):
        return self.parse_p(b), self.parse_t(b)
    def parse_p(self, b):
        return (int(b[2:16], 2) - 1.0 * self.OSdig) / (2 **14) * self.FSS
    def parse_t(self, b):
        return (int(b[16:-5], 2) * 200. / (2**11 -1) - 50)

class DlvrPressureSensor(PressureSensor):
    """
    This class reads from the DLVR Series Pressure Sensor from All Sensors.
    """
    def __init__(self, addr, FSP=0.5, differential=True, desc='DLVR Series Pressure Sensor'):#If this is a differential pressure sensor (True) or a gage pressure sensor (False)
        if differential:
            OSdig=8192
        else:
            OSdig=1638
        FSS = FSP * (differential + 1)
        super(DlvrPressureSensor, self).__init__(OSdig=OSdig, addr=addr, FSS=FSS, desc=desc)

class DlvPressureSensor(PressureSensor):
    """
    This class reads from the DLV Series Pressure Sensor from All Sensors.
    """
    def __init__(self, addr, FSP=0.5, desc='DLV Series Pressure Sensor'):
        super(DlvPressureSensor, self).__init__(OSdig=1638, addr=addr, FSS=FSP, desc=desc)
