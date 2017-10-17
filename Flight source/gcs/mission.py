# Simple test of branch and commit - Brandon
# Simple test #2 of branch and commit - Brandon

# This is how the mission is currently specified. Angles are in Radians. 
# You must re-run the GCS application to reload this file.
# Theta: 90 degrees is horizontal ( same altitude as boat )
# Phi: 0 degrees is straight behind the boat. This is always relative to the stern of the ship.
# This system can be replaced with a mission file or similiar. 
# THETA  = [1.2, 1.2, 1.4, 1.2,  1.3, 1.3, 1.3, 1.3]
# PHI    = [  0,  .5,   0, -.5,  -.5,   0,  .5,   0]
# L      = [ 50,  50,  50,  60,   60,  70,  70, 100] 
from math import pi
THETA  = [ 0, pi/4,  pi/4,  pi/4,  pi/4, ]
PHI    = [ 0,    0, +pi/4, -pi/4,     0, ]
L      = [50,   50,    50,    50,    50, ] 
