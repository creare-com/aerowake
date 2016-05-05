import Adafruit_ADS1x15 # sudo pip install adafruit-ads1x15

class TensionSensor:
    def __init__(self):
        self._adc = Adafruit_ADS1x15.ADS1115(0x48)
        self._c1 = 1; # Nominally 1/(2 sin (170deg / 2))
        self._c2 = 0; # nominally 0
    def readTension(self):
        """ Returns cable tension in newtons """
        # 0 means channel 0 minus channel 1
        adc_volts = self._adc.read_adc_difference(0, gain=1)
        return adc_volts * self._c1 + self._c2;

if __name__ == "__main__":
    ts = TensionSensor()
    while True:
        print("Tension = " + str(ts.readTension()))
    