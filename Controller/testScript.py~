import numpy as np

point1=[42.356998,-71.092478]
point2=[42.355696,-71.092334]
point3=[42.355805,-71.088758]
point4=[42.357030,-71.090754]

act1=point4
act2=point1

alt1=0
alt2=0


rShip=637100+alt1	
rUav=637100+alt2

dlat=act1[0]-act2[0]
dlon=act1[1]-act2[1]

R=6371229

a = (np.sin(dlat/2*np.pi/180))**2 + np.cos(act1[0]*np.pi/180) * np.cos(act2[0]*np.pi/180) * (np.sin(dlon/2*np.pi/180))**2

c = 2 * np.arctan2(np.sqrt(a),np.sqrt(1-a))
d=R * c

print "distance = "+ str(d)

##############################################################

theta = np.arctan2(       np.sin(dlon*np.pi/180) * np.cos(act2[0]*np.pi/180) , np.cos(act1[0]*np.pi/180) * np.sin(act2[0]*np.pi/180) - np.sin(act1[0]*np.pi/180) * np.cos(act2[0]*np.pi/180) * np.cos(dlon*np.pi/180))

print "theta_raw = "+ str(theta)

theta2 = -theta

if theta2<0:
	theta2=theta2+2*np.pi
theta2=theta2*180/np.pi

print "bearing = " + str( theta2)
