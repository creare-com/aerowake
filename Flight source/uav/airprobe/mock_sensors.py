""" 
Mock airprobe sensor objects for easier testing.
See non-mock versions of classes for proper documentation.
"""

class I2cSensor(object):
    def __init__(self, addr, desc='I2C Sensor', busnum=None):
        self._description = desc
    def get_desc(self):
        return self._description
    def set_desc(self, desc):
        self._description = desc

class TemperatureSensor(I2cSensor):
    def __init__(self, addr, desc='Temperature Sensor'):
        super(TemperatureSensor, self).__init__(addr=addr, desc=desc, busnum=None)
    def read_temp_c(self):
        return 0
    def read_temp_f(self):
        return 0
    def parse_t(self, b):
        return 0

class PressureSensor(I2cSensor):
    def __init__(self, addr, FSS=0.5, OSdig=1638, desc='Pressure Sensor'):
        super(PressureSensor, self).__init__(addr=addr, desc=desc, busnum=None)
    def retrieve_p(self):
        pass
    def retrieve_t(self):
        pass
    def read_p(self):
        return 0
    def read_t(self):
        return 0
    def parse_p(self, b = None):
        return 0
    def parse_t(self, b = None):
        return 0

class DlvrPressureSensor(PressureSensor):
    def __init__(self, addr, FSP=0.5, differential=True, desc='DLVR Series Pressure Sensor'):#If this is a differential pressure sensor (True) or a gage pressure sensor (False)
        super(DlvrPressureSensor, self).__init__(addr=addr, desc=desc, busnum=None)

class DlvPressureSensor(PressureSensor):
    def __init__(self, addr, FSP=0.5, desc='DLV Series Pressure Sensor'):
        super(DlvPressureSensor, self).__init__(addr=addr, desc=desc, busnum=None)
