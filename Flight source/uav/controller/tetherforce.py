import numpy as np

def tether_force(phi,theta,L):
    
    linear_density =  .001 
    n_max = 500
    e_thres = 0.01
    g = 9.81

    TETHER_GAIN = 1

    m_tether = linear_density*L*TETHER_GAIN


    if theta>1.45: # Limit force in horizontal direction
        theta=1.45

    Fz = m_tether*g
    Fh = Fz*np.tan(theta)
    Fx = Fh*np.cos(phi)
    Fy = Fh*np.sin(phi)
                    
    return [Fx,Fy,Fz]
            
            
            
            
            
            
            
            
            
            
            
