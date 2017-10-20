# The mission is specified in spherical coordinates with angles in radians and length in meters. Each of the lists below must be of the same length. 

# The convention is as follows:
#   The origin is defined as the GCS.
#   THETA : elevation angle defined from vertical : 90 degrees is horizontal (same altitude as GCS) : 0 degrees is vertical (directly above GCS)
#   PHI : azimuth angle defined from GCS longitudinal axis : 0 degrees is directly behind the GCS : 90 degrees is directly right of the GCS when facing in direction of GCS

# The general model of this file is: 
# THETA  = [1.2, 1.2, 1.4, 1.2,  1.3, 1.3, 1.3, 1.3] # [rad]
# PHI    = [  0,  .5,   0, -.5,  -.5,   0,  .5,   0] # [rad]
# L      = [ 50,  50,  50,  60,   60,  70,  70, 100] # [m]

# You must re-run the GCS main.py to reload this file.

from math import pi
THETA  = [ 0, pi/4,  pi/4,  pi/4,  pi/4, ] # [rad]
PHI    = [ 0,    0, +pi/4, -pi/4,     0, ] # [rad]
L      = [50,   50,    50,    50,    50, ] # [m]
