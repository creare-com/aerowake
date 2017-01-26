#!/bin/bash
sudo cp 99-ftdi.rules /etc/udev/rules.d/
# sudo cp libEposCmd.so.5.0.1.0 /usr/lib
# sudo cp libftd2xx.so.1.2.8 /usr/lib
# cd /usr/lib
sudo ln -s libEposCmd.so.5.0.1.0 libEposCmd.so
sudo ln -s libftd2xx.so.1.2.8 libftd2xx.so

sudo service udev restart

sudo pip install cython Adafruit_ADS1X15 enum

