import spidev

test_busses = [(1,0), (2,0), (2,1), (3,0), (3,1)]

for dev in test_busses:
    try:
        spi = spidev.SpiDev()
        print("Opening /dev/spidev%d.%d"%dev)
        bus,device = dev
        spi.open(*dev)
        outbound = [1,2,3,4]
        inbound = spi.xfer(outbound)
        spi.close()
        print("spi.xfer(outbound) results in: outbound: " + repr(outbound) + ", inbound: " + repr(inbound))
        
    except e:
        print("Caught: " + repr(e))
