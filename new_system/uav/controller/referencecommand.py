import numpy as np

def reference_command(theta,phi,L):
    
    linear_density = .04/3
    n_max = 500
    e_thres = 0.01
    a = 1
    R = L 
    
    if theta>1.4:
        theta_out = theta
        theta = 1.4
    else:
        theta_out = theta
        
    for i in range(0,n_max):


        I1 = a*(np.cosh((R**2 - R**2*np.cos(theta)**2)**(0.5)/a) - 1) - R*np.cos(theta)
        I2 = a*np.sinh((R**2 - R**2*np.cos(theta)**2)**(0.5)/a) - L

        J11 = (np.sinh((R**2 - R**2*np.cos(theta)**2)**(0.5)/a)*(2*R - 2*R*np.cos(theta)**2))/(2*(- R**2*np.cos(theta)**2 + R**2)**(0.5)) - np.cos(theta)
        J12 = np.cosh((R**2 - R**2*np.cos(theta)**2)**(0.5)/a) - (np.sinh((R**2 - R**2*np.cos(theta)**2)**(0.5)/a)*(R**2 - R**2*np.cos(theta)**2)**(0.5))/a - 1
        J21 = (np.cosh((R**2 - R**2*np.cos(theta)**2)**(0.5)/a)*(2*R - 2*R*np.cos(theta)**2))/(2*(- R**2*np.cos(theta)**2 + R**2)**(0.5))
        J22 = np.sinh((R**2 - R**2*np.cos(theta)**2)**(0.5)/a) - (np.cosh((R**2 - R**2*np.cos(theta)**2)**(0.5)/a)*(R**2 - R**2*np.cos(theta)**2)**(0.5))/a
        J_int = np.matrix([[J22, -J12],[  -J21, J11]])
        J_inv = (1/(J11*J22 -J12*J21))*J_int
        iter = np.matrix([[R],[a]]) - J_inv* np.matrix([[I1],[I2]])* .75
        
        R = iter[0,0]
        a = iter[1,0]
        
        e1 = a*(np.cosh((R**2 - R**2*np.cos(theta)**2)**(0.5)/a) - 1) - R*np.cos(theta)
        e2 = a*np.sinh((R**2 - R**2*np.cos(theta)**2)**(0.5)/a) - L
        e_iter = np.sqrt(e1**2 + e2**2);    
        if e_iter<e_thres:
            R_out = R
            a_out = a
            break

        a_max = R*50
        if a>a_max:
            break 
        
        
    return [theta_out,phi,R,i]
