#!/bin/bash

# /home/pi/FlightRecorder/FlightRecorder -p /dev/ttyS0

while [ 0 ] ; do
    # /home/pi/FlightRecorder/SpiSensorTest
    # python /home/pi/FlightRecorder/spi_test.py;
    # python /home/pi/FlightRecorder/spi_test_read_dlhr_status_cs0.py;
    python /home/pi/FlightRecorder/spi_test_set_mux_and_read.py;
    sleep 1;
done
