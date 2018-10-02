""" 

Contains classes to interface with sensors from the manufacturer called 'All Sensors'.
(Does not actually claim to contain all sensors)
"""
from I2cSensor import I2cSensor 


class PressureSensor(I2cSensor):
    """
    This class handles math and interactions common among All Sensors pressure sensors.
    """
    def __init__(self, addr, FSS=0.5, OSdig=1638, desc='Pressure Sensor', busnum=None):
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
        super(PressureSensor, self).__init__(addr=addr, desc=desc, busnum=busnum)
        # Save attributes
        self.FSS = FSS
        self.OSdig = OSdig
        self._retrieved_p_word = None
        self._retrieved_t_word = None
    
    def retrieve_p(self):
        """
        Reads pressure word from the sensor but does not parse it.
        Reads are split into "retrieve" and "parse" so that we can do all the sensor communications
        as close to simultaneously as possible.
        """
        try:
            self._retrieved_p_word = self._read_byte_data(0, 2)
        except IOError:
            self._retrieved_p_word = [0,0]
            print "Could not read pressure sensor at address %02X"%self._address
        
    def retrieve_t(self):
        """
        Reads temperature word from the sensor but does not parse it.
        Reads are split into "retrieve" and "parse" so that we can do all the sensor communications
        as close to simultaneously as possible.
        """
        self._retrieved_t_word = self._read_byte_data(2, 2)
        
    def read_p(self):
        """
        Retrieves the pressure, parses it, and returns the result.
        
        Returns
        -------
        pressure : float
            The pressure indicated by the sensor, in the same units that FSS was provided to __init__ earlier
        """
        self.retrieve_p()
        return self.parse_p()
        
    def read_t(self):
        """
        Retrieves the temperature, parses it, and returns the result.
        
        Returns
        -------
        temperature : float
            The temperature indicated by the sensor, in degrees C
        """
        self.retrieve_t()
        return self.parse_t()
        
    def parse_p(self, b = None):
        """
        Parameters
        ----------
        b : list of two integers, each < 255
            If provided, will parse this word.  Otherwise, will parse the last retrieved data.
            Throws an exception if you haven't provided b or called retrieve_p.
        
        Returns
        -------
        pressure : float
            The pressure indicated by the sensor, in the same units that FSS was provided to __init__ earlier,
            as of the last time retrieve_p was called.
        """
        if not b:
            b = self._retrieved_p_word
            if not b:
                raise Exception('No data to parse!  Make sure to call retrieve_p() or supply a value to parse.')
        p_word = self._two_bytes_to_one_word(b) & 0x3FFF
        return 1.25 * (p_word - 1.0 * self.OSdig) / (2 **14) * self.FSS

    def parse_t(self, b = None):
        if not b:
            b = self._retrieved_t_word
            if not b:
                raise Exception('No data to parse!  Make sure to call retrieve_t() or supply a value to parse.')
        t_word = (self._two_bytes_to_one_word(b) & 0xFFE0) >> 5
        return (t_word * 200.0 / (2**11 -1) - 50)

class DlvrPressureSensor(PressureSensor):
    """
    This class reads from the DLVR Series Pressure Sensor from All Sensors.
    """
    def __init__(self, addr, FSP=0.5, differential=True, desc='DLVR Series Pressure Sensor', busnum=None):#If this is a differential pressure sensor (True) or a gage pressure sensor (False)
        if differential:
            OSdig=8192
        else:
            OSdig=1638
        FSS = FSP * (differential + 1)
        super(DlvrPressureSensor, self).__init__(OSdig=OSdig, addr=addr, FSS=FSS, desc=desc, busnum=busnum)

class DlvPressureSensor(PressureSensor):
    """
    This class reads from the DLV Series Pressure Sensor from All Sensors.
    """
    def __init__(self, addr, FSP=0.5, desc='DLV Series Pressure Sensor', busnum=None):
        super(DlvPressureSensor, self).__init__(OSdig=1638, addr=addr, FSS=FSP, desc=desc, busnum=busnum)
