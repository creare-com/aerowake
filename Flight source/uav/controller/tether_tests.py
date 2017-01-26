

from referencecommand import reference_command
from feedforward import feed_forward
import numpy as np

# phi_in= -1
# th_in = 0

# phi = 0  * np.pi/180
# th = 45   * np.pi/180

# fix = -phi_in*np.sin(phi) + th_in*np.cos(th)*np.cos(phi)
# fiy = phi_in*np.cos(phi) + th_in*np.cos(th)*np.sin(phi)
# fiz = -th_in*np.sin(th) 
# print "\n \n"
# print "%.2f  %.2f  %.2f"  %(fix,fiy,fiz)
# print "\n \n"
# exit()

#output = reference_command(1.5,0,20);
output = feed_forward(0,1.4,20.0,20.0,22.0)
print output






[0.053701452887172874, 1.450062900759018, 82.779582809360718, 49.765601451762493, 0]
