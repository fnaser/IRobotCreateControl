from math import *
import numpy as np

'''
let the vehicle have two wheels with speeds U1 and U2. lets U1 be the one on the left and U2 the one on the right when looking form above with the front of hte vehicle
            X_car
           / \
            |
            |
------|<----o---->|-------> Y_car
     U1     |    U2
            |
            |
           \ /


V_cm = 0.5*(U1+U2) = sqrt( X_dot^2 + Y_dot^2 ) 

Theta_dot = 1/ro * 1/2 * (U2-U1)

X_dot = BU
 _         _       _                              _     _  _
| X_dot     |     | 1/2*cos(theta), 1/2*cos(theta) | * | U1 |
| Y_dot     |   = | 1/2*sin(theta), 1/2*sin(theta) |   |_U2_|
|_Theta_dot_|     |_-1/(2*ro)     , 1/(2*ro)      _|

X_k+1 = I+Bu

Inverting V_cm and theta_dot eq we get

U1 = V-R*theta_dot
U2 = V+R*theta_dot
'''

def UkFromXkanXkplusone(Xk,Xkp1,ro,dt):
    V = sqrt(  ((Xkp1[0]-Xk[0])/dt)**2  + ((Xkp1[1]-Xk[1])/dt)**2 )
    thetadot = (Xkp1[2]-Xk[2])/dt
    Uk = [V-ro*thetadot, V+ro*thetadot]
    return np.array(Uk) 


def TrajToUko(Xks,ro,dt):
    Ukos = []
    for i in range(0,len(Xks)-1):
        Xk = Xks[i]
        Xkp1 = Xks[i+1]
        Ukos.append(np.array(UkFromXkanXkplusone(Xk,Xkp1,ro,dt)))
    return Ukos


def TVLQR(xtraj, utraj, dt, r0, Q, R):
    Ktraj = []
    S = np.matrix(Q)
    Q = np.matrix(Q)
    R = np.matrix(R)
    for k in range(len(xtraj)-1,-1,-1):
        Bk = B(xtraj(k),r0)
        K = -(R + Bk.T*S*Bk).I*Bk.T*S
        S = Q + K.T*R*K + (np.matrix(np.identity(3)) + Bk*K).T*S*(np.matrix(np.identity(3)) + Bk*K)
        Ktraj.append(K)
    return Ktraj


def B(x,r0):
    th = x[2]
    B = np.matrix([[.5*cos(th), .5*cos(th)],[.5*sin(th), .5*sin(th)],[-1/(2*r0), 1/(2*r0)]])
    return B


