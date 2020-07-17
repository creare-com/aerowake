#!/bin/bash
sudo cp 99-ftdi.rules /etc/udev/rules.d/
# sudo cp libEposCmd.so.5.0.1.0 /usr/lib
# sudo cp libftd2xx.so.1.2.8 /usr/lib
CALLER_PATH=`pwd`
cd "`dirname "$0"`"
MY_PATH=`pwd`
cd "$CALLER_PATH"
echo "My path is $MY_PATH, being called from $CALLER_PATH"
sudo ln -s "$MY_PATH/libEposCmd.so.5.0.1.0" /usr/lib/libEposCmd.so
sudo ln -s "$MY_PATH/libftd2xx.so.1.2.8" /usr/lib/libftd2xx.so

sudo service udev restart

sudo pip3 install cython Adafruit_ADS1X15

