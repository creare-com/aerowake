"""
Assumes a DLHR pressure sensor from AllSensors is connected to SPI0, CS0.

Attempts to read the status word once.

"""
import spidev
import time

dev = (0,0)
sleep_millis = 8 # Technically only need to sleep 4ms.  Sleeping longer to be sure.

try:
    spi = spidev.SpiDev()
    print("Opening /dev/spidev%d.%d"%dev)
    bus,device = dev
    spi.open(*dev)
    spi.max_speed_hz = 5000
    print("spi.bits_per_word: %d, spi.cshigh: %d, spi.loop: %d, spi.no_cs: %d, spi.lsbfirst: %d, spi.max_speed_hz: %d, spi.mode: %d, spi.threewire: %d"%(spi.bits_per_word, spi.cshigh, spi.loop, spi.no_cs, spi.lsbfirst, spi.max_speed_hz, spi.mode, spi.threewire))
    outbound = [0xAA, 0, 0]
    print("Sending: " + repr(outbound))
    inbound = spi.xfer(outbound)
    print("Received: " + repr(inbound))
    print("Sleeping for %dms."%sleep_millis);
    time.sleep(sleep_millis * 0.001)
    outbound = [0, 0, 0, 0, 0, 0, 0]
    print("Sending: " + repr(outbound))
    inbound = spi.xfer(outbound)
    print("Received: " + repr(inbound))
    
    
except Exception as e:
    print("Caught: " + repr(e))
finally:
    print("Closing.")
    spi.close()
