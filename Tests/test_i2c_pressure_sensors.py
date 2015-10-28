import smbus
import Adafruit_GPIO.I2C as I2C

def parse_bits(a):
    b = ''.join(['0'*(8-aa.bit_length())+ bin(aa)[2:] for aa in a])
    return b


class Sensor(object):
    def __init__(self, address, FSS=0.5, differential=True):
        """
        Parameters
        -----------
        address : int
            i2c sensor address. Can be specified as (for example) 0x10
        FSS : float
            Sensor full scale level. span in inH2O
        differential : bool
            If this is a differential pressure sensor (True) or a guage pressure sensor (False)
        """
        # Save attributes
        self.address = address
        self.FSS = FSS
        self.differential = differential
        if differential:
            self.OSdig = 8192
        else:
            self.OSdig = 1638

	# Open the bus
        busnum = I2C.get_default_bus()
        self._bus = smbus.SMBus(busnum)
      
    def read(self):
        a = self._bus.read_i2c_block_data(self.address, 0, 4)
        b = parse_bits(a)
        return self.parse_pt(b)
    def parse_pt(self, b):
        return self.parse_p(b), self.parse_t(b)
    def parse_p(self, b):
        return (int(b[2:16], 2) - 1.0 * self.OSdig) / (2 **14) * self.FSS * (self.differential + 1)
    def parse_t(self, b):
        return (int(b[16:-5], 2) * 200. / (2**11 -1) - 50)
    def __del__(self):
        self._bus.close()    
        
if __name__ == "__main__":
    import time
    t0 = time.time()
    s1 = Sensor(0x10)
    s2 = Sensor(0x20, FSS=5)
    l = []
    for i in xrange(100):
        l = (time.time()-t0, s1.read(), time.time()-t0, s2.read())
        print "S1:", l[0], l[1][0],
        print "\tS2: ", l[2],  l[3][0]
        
