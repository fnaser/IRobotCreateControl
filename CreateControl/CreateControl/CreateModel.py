﻿from math import *
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

def minAngleDif(x,y):
     a = atan2(sin(x-y), cos(x-y))
     #if(a<0):a = 2*pi-a
     return a

def UkFromXkandXkplusone(Xk,Xkp1,ro,dt):
    V = sqrt(  ((Xkp1[0]-Xk[0])/dt)**2  + ((Xkp1[1]-Xk[1])/dt)**2 )
    #http://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    d_theta = atan2(sin(Xkp1[2]-Xk[2]), cos(Xkp1[2]-Xk[2]))
    thetadot = d_theta/dt
    gain = 1.0328
    Uk = [gain*(V-ro*thetadot), gain*(V+ro*thetadot)]
    return np.array(Uk) 


def TrajToUko(Xks,ro,dt):
    Ukos = []
    for i in range(0,len(Xks)-1):
        Xk = Xks[i]
        Xkp1 = Xks[i+1]
        Ukos.append(np.array(UkFromXkandXkplusone(Xk,Xkp1,ro,dt)))
    for i in range(len(Ukos)-1,0,-1):
        Uk = Ukos[i]
        if Uk[0] == 0 and Uk[1]==0:
            Ukos[i] = Ukos[i+1]
    return Ukos


def TVLQR(xtraj, utraj, dt, r0, Q, R):
    '''Q should be diag([1/distance deviation ^2, 1/speed deviation ^2])
       R should be 1/ control deviation^2
    '''
    Ktraj = []
    ia = np.mat(np.eye(3))
    ib = dt*ia
    za = np.mat(np.zeros((3,3)))

    A = np.bmat([[ia, ib],[za, za]])
    S = np.matrix(Q)
    Q = np.matrix(Q)
    R = np.matrix(R)
    for k in range(len(xtraj)-1,-1,-1):
        Bk = B(xtraj[k][0],r0)
        K = (R + Bk.T*S*Bk).I*Bk.T*S
        S = Q + K.T*R*K + (np.matrix(A - Bk*K).T*S*(A - Bk*K))
        Ktraj.append(K)

    Ktraj_output=[]
    for K in reversed(Ktraj):
        Ktraj_output.append(K)
    return Ktraj_output


def B(th,r0):
    '''returns the B matrix
       this expects a 1d array for X
    '''
    #th = x[2,0]
    #print th
    g = .81
    za = np.mat(np.zeros((3,2)))
    Ba = np.mat([[g*.5*cos(th), g*.5*cos(th)],
                 [g*.5*sin(th), g*.5*sin(th)],
                 [g/(2.0*r0), -g/(2.0*r0)]] )

    B = np.bmat([[za],
                 [Ba]])
    return B


def A(dt):
    Ia = np.mat(np.eye(3))
    Dt = dt*Ia
    Za = np.mat(np.zeros((3,3)))
    A = np.bmat([[Ia,Dt],[Za,Za]])
    return A

