import airspeed_sensor
import time

airspeed = airspeed_sensor.airspeed_sensor(0x28)

time.sleep(.5)

while True:
    print airspeed.read()[0]
    time.sleep(.1)
