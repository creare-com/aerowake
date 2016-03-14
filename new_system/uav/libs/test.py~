from Quaternion import Quat
import numpy as np

roll = 0
pitch = 30
yaw = 45

e1 = yaw   # Z
e2 = -pitch # Y
e3 = roll  # X

quat = Quat((e1,e2,e3))

q1 = quat.q[0]
q2 = quat.q[1]
q3 = quat.q[2]
q0 = quat.q[3]


roll = np.arctan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
pitch = np.arcsin(2*(q0*q2-q3*q1))
yaw = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))

print [q0,q1,q2,q3]
print (roll*180/np.pi,pitch*180/np.pi,yaw*180/np.pi)




c1=np.cos(yaw/2)
c2=np.cos(pitch/2)
c3=np.cos(roll/2)

s1=np.sin(yaw/2)
s2=np.sin(pitch/2)
s3=np.sin(roll/2)

w = c1*c2*c3-s1*s2*s3
x = s1*s2*c3+c1*c2*s3
y = s1*c2*c3+c1*s2*s3
z = c1*s2*s3-s1*c2*s3
print [w,x,y,z]

c1=np.cos(yaw)
c2=np.cos(pitch)
c3=np.cos(roll)

s1=np.sin(yaw)
s2=np.sin(pitch)
s3=np.sin(roll)

w = np.sqrt(1.0 + c1 * c2 + c1*c3 - s1 * s2 * s3 + c2*c3) / 2
x = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / (4.0 * w) 
y = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / (4.0 * w)
z = (-s1 * s3 + c1 * s2 * c3 + s2) /(4.0 * w) 
print [w,x,y,z]

