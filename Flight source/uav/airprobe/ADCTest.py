import time, signal, sys, datetime
from Adafruit_ADS1x15 import ADS1x15


def signal_handler(signal, frame):
        print 'You pressed Ctrl+C!'
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

############### SETUP #################################################

ADS1015 = 0x00  # 12-bit ADC

# Select the gain
gain = 6144  # +/- 6.144V
# gain = 4096  # +/- 4.096V
# gain = 2048  # +/- 2.048V
# gain = 1024  # +/- 1.024V
# gain = 512   # +/- 0.512V
# gain = 256   # +/- 0.256V

# Select the sample rate
# sps = 8    # 8 samples per second
# sps = 16   # 16 samples per second
# sps = 32   # 32 samples per second
# sps = 64   # 64 samples per second
# sps = 128  # 128 samples per second
sps = 250  # 250 samples per second
# sps = 475  # 475 samples per second
# sps = 860  # 860 samples per second

# Initialise the ADC using the default mode (use default I2C address)
# Set this to ADS1015 or ADS1115 depending on the ADC you are using!


adc0 = ADS1x15(ic=ADS1015,address=0x48)
#adc1 = ADS1x15(ic=ADS1015,address=0x49)

############### MAIN ###############################################

while True:
	f=open('TempData.txt','a')
	Ch00 = adc0.readADCSingleEnded(0,gain,sps)/1000
	Ch01 = adc0.readADCSingleEnded(1,gain,sps)/1000
	Ch02 = adc0.readADCSingleEnded(0,gain,sps)/1000
	Ch03 = adc0.readADCSingleEnded(1,gain,sps)/1000

	#print "%.4f    %.4f    %.4f    %.4f" % (Ch00,Ch01,Ch02,Ch03)

	now = datetime.datetime.now()
	timestamp = now.strftime("%H:%M:%S")
	printstr = "%s   %.4f    %.4f    %.4f    %.4f \n" % (timestamp,Ch00,Ch01,Ch02,Ch03)
	print printstr	
	f.write(printstr)
	f.close()
	time.sleep(.5)









