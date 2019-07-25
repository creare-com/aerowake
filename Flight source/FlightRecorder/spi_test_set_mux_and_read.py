"""
Assumes a DLHR pressure sensor from AllSensors is connected to SPI0, CS0.

Attempts to read the status word once.

"""
import spidev

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
    

try:
    print("Opening mux port.")
    muxPort = spidev.SpiDev()
    muxPort.open(0, 0)
    muxPort.max_speed_hz = 5000
    print("Opening sensor port.")
    sensorPort = spidev.SpiDev()
    sensorPort.open(0, 1)
    sensorPort.max_speed_hz = 5000
    # print("spi.bits_per_word: %d, spi.cshigh: %d, spi.loop: %d, spi.no_cs: %d, spi.lsbfirst: %d, spi.max_speed_hz: %d, spi.mode: %d, spi.threewire: %d"%(spi.bits_per_word, spi.cshigh, spi.loop, spi.no_cs, spi.lsbfirst, spi.max_speed_hz, spi.mode, spi.threewire))
    print("Setting mux to port 1 (counting from 1).")
    send(muxPort, [0x00])
    print("Reading DLHR status.")
    send(sensorPort, [0xF0])
    print("Setting mux to port 14 (counting from 1).")
    send(muxPort, [0x0D])
    print("Reading MAX6682 value.")
    send(sensorPort, [0x00, 0x00])
    print("Setting mux to port 13 (counting from 1).")
    send(muxPort, [0x0C])
    print("Reading DLV value.")
    send(sensorPort, [0x00, 0x00, 0x00, 0x00])
    print("Setting mux to port 2 (counting from 1).")
    send(muxPort, [0x01])
    
except Exception as e:
    print("Caught: " + repr(e))
finally:
    print("Closing.")
    muxPort.close()
    sensorPort.close()
