import time
api, gcs = local_connect()
vehicle = api.get_vehicles()[0]

#def mavrx_debug_handler(message):
#    print "SomethingRandom"
#    print "Received", message

#vehicle.set_mavlink_callback(mavrx_debug_handler)
time.sleep(10)
print "DONE WITH API STUFF"
stop



