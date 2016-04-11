import numpy as np

def feed_forward(phi,theta,R,R_ref,L):
    
    linear_density =  .04/3 
    n_max = 500
    e_thres = 0.01
    g = 9.81
    a = 1
    b = 0

    Xin = R*np.sin(theta)*np.cos(phi)
    Yin = R*np.sin(theta)*np.sin(phi)
    Z = R*np.cos(theta)
    
    #Check to make sure the reference command is inside L
    if R_ref>(0.99999*L):
        print "Reference Broken: R_ref=",R_ref
        R_ref=.98*L
        
    
    #If R is outside reference, feed forward the force to stay at R_ref
    if R>R_ref:
        Xin=Xin*R_ref/R;
        Yin=Yin*R_ref/R;
        Z=  Z*R_ref/R;

		
        
        
    x = np.sqrt(Xin**2 + Yin**2)
    X=x

    for i in range(0,n_max):
        
        y1 = a*( np.cosh((X+b)/a) -np.cosh(b/a) )-Z
        y2 = -a* np.sqrt(np.cosh(b/a)**2) * np.tanh(b/a)+a*np.sqrt(1+np.sinh((b+X)/a)**2)* np.tanh((b+X)/a) -L

        J11 = np.cosh((b + x)/a) - np.cosh(b/a) - a*((np.sinh((b + x)/a)*(b + x))/a**2 - (b*np.sinh(b/a))/a**2)
        J12 = -a*(np.sinh(b/a)/a - np.sinh((b + x)/a)/a)
        J21 = 2*np.sinh(x/(2*a))*np.cosh((2*b + x)/(2*a)) - (x*np.cosh((2*b + x)/(2*a))*np.cosh(x/(2*a)))/a - (np.sinh(x/(2*a))*np.sinh((2*b + x)/(2*a))*(2*b + x))/a
        J22 = 2*np.sinh(x/(2*a))*np.sinh((2*b + x)/(2*a))
        J_int = np.matrix([[J22, -J12],[  -J21, J11]])
        J_inv = (1/(J11*J22 -J12*J21))*J_int
        iter = np.matrix([[a],[b]]) - J_inv* np.matrix([[y1],[y2]])* .3
        
        a = iter[0,0]
        b = iter[1,0]
        
        e1 = a*( np.cosh((X+b)/a) -np.cosh(b/a) )-Z
        e2 = -a* np.sqrt(np.cosh(b/a)**2) * np.tanh(b/a)+a*np.sqrt(1+np.sinh((b+X)/a)**2)* np.tanh((b+X)/a) -L
        e_iter = np.sqrt(e1**2 + e2**2)
        if e_iter<e_thres:
            a_out = a
            b_out = b
            break
     
    xc = -b
    l_total =  a *(np.sqrt(1+np.sinh((b+X)/a)**2)* np.tanh((b+X)/a)-np.sqrt(1+np.sinh((b+xc)/a)**2) * np.tanh((b+xc)/a))
    m_total = l_total*linear_density
    
    alpha = np.arctan(np.sinh((b+X)/a) )
    
    Ft = m_total*g/np.sin(alpha)
    Fz = m_total*g
    gamm = np.arctan(Yin/Xin)
    Fx = Ft*np.cos(alpha)*np.cos(gamm)
    Fy = Ft*np.cos(alpha)*np.sin(gamm)
                    
    return [-Fx,-Fy,Fz]
            
            
            
            
            
            
            
            
            
            
            
