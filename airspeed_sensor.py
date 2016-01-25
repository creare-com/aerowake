import time
from Adafruit_I2C import Adafruit_I2C

class airspeed_sensor:
    def __init__(self, adrs=None):
        self.i2c = Adafruit_I2C(adrs)
        self.address = adrs
        self.FSS = 0.5*100
        self.OSdig = 8192
        self.differential = True
        self.cal = 0

        self.calibrate()

        print 'Setup Complete'

	
    def parse_bits(self,a):
        b = ''.join(['0'*(8-aa.bit_length())+ bin(aa)[2:] for aa in a])
        return b
    def parse_pt(self, b):
        return self.parse_p(b), self.parse_t(b)
    def parse_p(self, b):
        return (int(b[2:16], 2) - 1.0 * self.OSdig) / (2 **14) * self.FSS * (self.differential + 1)
    def parse_t(self, b):
        return (int(b[16:-5], 2) * 200. / (2**11 -1) - 50)

    def calibrate(self):
        sum_p=0
        for i in range(1,100):
            cal_d = self.read()[0]
            sum_p += cal_d
            time.sleep(.005)
        self.cal = sum_p/100
        return 0

    def read(self):
        a = self.i2c.readList(0,4)   
        b = self.parse_bits(a)
        c=  self.parse_pt(b)

        d = c[0]-self.cal
        return [d,c[1]]

if __name__ == "__main__":
    import time
    airspeed = airspeed_sensor(0x28)

    time.sleep(.5)

    while True:
        print airspeed.read()[0]
        time.sleep(.1)




#    def read_measurement(self):
  #      data = self.i2c.readList(0,4)   
    #    out = self.bit_to_int(data[0],data[1])
#        print out
  #      return out
	

