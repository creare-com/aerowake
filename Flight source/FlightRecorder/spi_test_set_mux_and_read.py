"""
Assumes a DLHR pressure sensor from AllSensors is connected to SPI0, CS0.

Attempts to read the status word once.

"""
import spidev
import time
sleep_millis = 8 # Technically only need to sleep 4ms.  Sleeping longer to be sure.

def reprHex(intArr):
    """
    intArr: a list of ints 0-255
    return: string like '[0x01, 0x02, 0x03, 0x04]'
    """
    return '[' + (', '.join(["0x%02X"%c for c in intArr])) + ']'
    
def send(spiDevice, outbound):
    """
    spiDevice: an open spidev.SpiDev instance
    outbound: a list of ints to transmit, 0-255.  This function may modify this variable.
    """
    print("Sending: " + reprHex(outbound))
    inbound = spiDevice.xfer(outbound)
    print("Received: " + reprHex(inbound))
    
def printData(spi, name):
    """
    Prints a bunch of data
    spi: an open spidev.SpiDev instance
    name: human-readable description
    """
    print("%s: spi.bits_per_word: %d, spi.cshigh: %d, spi.loop: %d, spi.no_cs: %d, spi.lsbfirst: %d, spi.max_speed_hz: %d, spi.mode: %d, spi.threewire: %d"%(name, spi.bits_per_word, spi.cshigh, spi.loop, spi.no_cs, spi.lsbfirst, spi.max_speed_hz, spi.mode, spi.threewire))
    

try:
    print("Opening mux port.")
    muxPort = spidev.SpiDev()
    muxPort.open(0, 0)
    muxPort.mode = 1
    muxPort.max_speed_hz = 5000
    printData(muxPort, "muxPort")
    print("Opening sensor port.")
    sensorPort = spidev.SpiDev()
    sensorPort.open(0, 1)
    sensorPort.max_speed_hz = 5000
    printData(sensorPort, "sensorPort")
    
    for n in range(0, 12):
        print("----------------")
        print("Setting mux to port %d (counting from 1)."%(n+1))
        send(muxPort, [n])
        print("Reading DLHR status.")
        send(sensorPort, [0xF0])
        print("Commanding DLHR read.")
        send(sensorPort, [0xAA, 0, 0])
        print("Sleeping for %dms."%sleep_millis);
        time.sleep(sleep_millis * 0.001)
        print("Reading from DLHR.")
        send(sensorPort, [0, 0, 0, 0, 0, 0, 0])
    print("----------------")
    print("Setting mux to port 13 (counting from 1).")
    send(muxPort, [0x0C])
    print("Reading DLV value.")
    send(sensorPort, [0x00, 0x00, 0x00, 0x00])
    print("----------------")
    print("Setting mux to port 14 (counting from 1).")
    send(muxPort, [0x0D])
    print("Reading MAX6682 value.")
    send(sensorPort, [0x00, 0x00])
    
except Exception as e:
    print("Caught: " + repr(e))
finally:
    print("Closing.")
    muxPort.close()
    sensorPort.close()
