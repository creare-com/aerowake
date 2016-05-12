import Adafruit_ADS1x15 # sudo pip install adafruit-ads1x15

class TensionSensor:
    def __init__(self, adc_per_ct=0.0045203, adc_baseline=7848):
        self._adc = Adafruit_ADS1x15.ADS1115(0x48)
        self._N_PER_ADC_COUNT        = adc_per_ct; # Newtons per ADC count.  Includes factor of 1/(2 sin (170deg / 2))
        self._SENSOR_BASELINE_COUNTS = adc_baseline; # The sensor returns this ADC value when under no tension

    def readTension(self):
        """ Returns cable tension in newtons """
        # 0 means channel 0 minus channel 1
        adc_counts = self._adc.read_adc_difference(0, gain=1)
        adc_counts -= self._SENSOR_BASELINE_COUNTS
        return adc_counts * self._N_PER_ADC_COUNT;

if __name__ == "__main__":
    ts = TensionSensor()
    while True:
        print("Tension = " + str(ts.readTension()))
    
