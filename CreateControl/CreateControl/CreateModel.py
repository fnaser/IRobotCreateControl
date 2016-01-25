from math import *

def UkFromXkanXkplusone(Xk,Xkp1,ro,dt):
    V = sqrt(  ((Xkp1[0]-Xk[0])/dt)**2  + ((Xkp1[1]-Xk[1])/dt)**2 )
    thetadot = (Xkp1[2]-Xk[2])/dt
    Uk = [V-ro*thetadot, V+ro*thetadot]
    return Uk 


def TrajToUko(Xks,ro,dt):
    Ukos = []
    for i in range(0,len(Xks)-1):
        Xk = Xks[i]
        Xkp1 = Xks[i+1]
        Ukos.append(UkFromXkanXkplusone(Xk,Xkp1,ro,dt))
    return Ukos

