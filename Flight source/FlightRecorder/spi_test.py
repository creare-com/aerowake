import spidev

test_busses = [(0,0), (0,1)]

for dev in test_busses:
    try:
        spi = spidev.SpiDev()
        print("Opening /dev/spidev%d.%d"%dev)
        bus,device = dev
        spi.open(*dev)
        spi.max_speed_hz = 5000
        print("spi.bits_per_word: %d, spi.cshigh: %d, spi.loop: %d, spi.no_cs: %d, spi.lsbfirst: %d, spi.max_speed_hz: %d, spi.mode: %d, spi.threewire: %d"%(spi.bits_per_word, spi.cshigh, spi.loop, spi.no_cs, spi.lsbfirst, spi.max_speed_hz, spi.mode, spi.threewire))
        # while True:
        if True:
            # outbound = [1,2,3,4]
            outbound = [0,0,0,0,0,0,0,0]
            inbound = spi.xfer(outbound)
            print("spi.xfer(outbound) results in: outbound: " + repr(outbound) + ", inbound: " + repr(inbound))
        # inbound = spi.readbytes(4)
        # print("spi.readbytes(4) results in: inbound: " + repr(inbound))
        # oldval = spi.cshigh
        # spi.cshigh = True
        # print("spi.bits_per_word: %d, spi.cshigh: %d, spi.loop: %d, spi.no_cs: %d, spi.lsbfirst: %d, spi.max_speed_hz: %d, spi.mode: %d, spi.threewire: %d"%(spi.bits_per_word, spi.cshigh, spi.loop, spi.no_cs, spi.lsbfirst, spi.max_speed_hz, spi.mode, spi.threewire))
        # spi.cshigh = oldval
        # print("spi.bits_per_word: %d, spi.cshigh: %d, spi.loop: %d, spi.no_cs: %d, spi.lsbfirst: %d, spi.max_speed_hz: %d, spi.mode: %d, spi.threewire: %d"%(spi.bits_per_word, spi.cshigh, spi.loop, spi.no_cs, spi.lsbfirst, spi.max_speed_hz, spi.mode, spi.threewire))
        
    except Exception as e:
        print("Caught: " + repr(e))
    finally:
        print("Closing.")
        spi.close()