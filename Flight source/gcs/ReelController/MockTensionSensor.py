""" A simple mock tension sensor that simply returns 0 every call. """

class MockTensionSensor:
    def __init__(self, adc_per_ct=0.0045203, adc_baseline=7991):
        self._N_PER_ADC_COUNT        = adc_per_ct; # Newtons per ADC count.  Includes factor of 1/(2 sin (170deg / 2))
        self._SENSOR_BASELINE_COUNTS = adc_baseline; # The sensor returns this ADC value when under no tension
        self._tension = 0;

    def readTension(self):
        return self._tension

    def setTension(self, tension):
        """ Only for use in the mock system """ 
        self._tension = tension