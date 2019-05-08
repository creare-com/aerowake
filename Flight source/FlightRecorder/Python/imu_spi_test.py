#!/usr/bin/env python

# Requires py-spidev, please run the following:
#    sudo apt-get update 
#    sudo apt-get install python-dev
#    git clone git://github.com/doceme/py-spidev
#    cd py-spidev
#    sudo python setup.py install
import spidev

s=spidev.SpiDev()
s.open(3,0)
(dummy,ahx, alx, ahy, aly, ahz, alz) = s.xfer2([0xBB, 0,0, 0,0, 0,0])
(ahx*256+alx, ahy*256+aly, ahz*256+alz)
