
import numpy as np



def eul2quat(roll,pitch,yaw):

    cr2 = np.cos(roll*0.5);
    cp2 = np.cos(pitch*0.5);
    cy2 = np.cos(yaw*0.5);
    sr2 = np.sin(roll*0.5);
    sp2 = np.sin(pitch*0.5);
    sy2 = np.sin(yaw*0.5);

    q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
    q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
    q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
    q4 = cr2*cp2*sy2 - sr2*sp2*cy2;

    return [q1,q2,q3,q4]

print eul2quat(0,45*np.pi/180,45*np.pi/180)







# roll = 0
# pitch = 30
# yaw = 45

# e1 = yaw   # Z
# e2 = -pitch # Y
# e3 = roll  # X

# quat = Quat((e1,e2,e3))

# q1 = quat.q[0]
# q2 = quat.q[1]
# q3 = quat.q[2]
# q0 = quat.q[3]


# roll = np.arctan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
# pitch = np.arcsin(2*(q0*q2-q3*q1))
# yaw = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))

# print [q0,q1,q2,q3]
# print (roll*180/np.pi,pitch*180/np.pi,yaw*180/np.pi)




# c1=np.cos(yaw/2)
# c2=np.cos(pitch/2)
# c3=np.cos(roll/2)

# s1=np.sin(yaw/2)
# s2=np.sin(pitch/2)
# s3=np.sin(roll/2)

# w = c1*c2*c3-s1*s2*s3
# x = s1*s2*c3+c1*c2*s3
# y = s1*c2*c3+c1*s2*s3
# z = c1*s2*s3-s1*c2*s3
# print [w,x,y,z]

# c1=np.cos(yaw)
# c2=np.cos(pitch)
# c3=np.cos(roll)

# s1=np.sin(yaw)
# s2=np.sin(pitch)
# s3=np.sin(roll)

# w = np.sqrt(1.0 + c1 * c2 + c1*c3 - s1 * s2 * s3 + c2*c3) / 2
# x = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / (4.0 * w) 
# y = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / (4.0 * w)
# z = (-s1 * s3 + c1 * s2 * c3 + s2) /(4.0 * w) 
# print [w,x,y,z]

