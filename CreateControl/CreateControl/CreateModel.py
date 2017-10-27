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
    #V = sqrt(  ((Xkp1[0]-Xk[0])/dt)**2  + ((Xkp1[1]-Xk[1])/dt)**2 )
    #http://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    #d_theta = minAngleDif(Xkp1[2],Xk[2])
    #thetadot = d_theta/dt
    
    DX = Xkp1-Xk
    DX[2] = minAngleDif(Xkp1[2],Xk[2])
    Binv = 1.0/dt*PseudoBInv(Xk[2],ro)

    #Uk = [(V-ro*thetadot),(V+ro*thetadot)]
    #Uk = np.array(Uk) 

    #Uk = np.array([[(V+ro*thetadot)],
    #               [(V-ro*thetadot)]])
    
    Uk = Binv.dot(DX)
    G,Mo = MotorGainAndOffset()
    GI = np.linalg.inv(G)
    Uc = GI.dot(Uk.transpose()-Mo)

    return  Uc.transpose()


def TrajToUko(Xks,ro,dt):
    Ukos = []
    for i in range(0,len(Xks)-1):
        Xk = Xks[i]
        Xkp1 = Xks[i+1]
        Ukos.append(np.array(UkFromXkandXkplusone(Xk,Xkp1,ro,dt)))
    #for i in range(len(Ukos)-1,0,-1):
    #    Uk = Ukos[i]
    #    if Uk[0] == 0 and Uk[1]==0:
    #        Ukos[i] = Ukos[i+1]
    return Ukos


def TVLQR(xtraj, utraj, dt, r0, Q, R):
    '''Q should be diag([1/distance deviation ^2, 1/speed deviation ^2])
       R should be 1/ control deviation^2
    '''
    Ktraj = []
    ia = np.mat(np.eye(3))

    Ak = A(dt)
    S = np.matrix(Q)
    Q = np.matrix(Q)
    R = np.matrix(R)
    G,V = MotorGainAndOffset()
    #GI = np.linalg.inv(G)

    for k in range(len(xtraj)-1,-1,-1):
        Bk = B(xtraj[k][0],r0).dot(G)
        K = -(R + Bk.T*S*Bk).I*Bk.T*S #-(R + Bk.T*S*Bk).I*Bk.T*S
        S = Q + K.T*R*K + (np.matrix(np.identity(3)) + Bk*K).T*S*(np.matrix(np.identity(3)) + Bk*K)
        Ktraj.append(K)

    Ktraj_output=[]
    for K in reversed(Ktraj):
        Ktraj_output.append(K)
    return Ktraj_output


def B(th,r0):
    '''returns the B matrix
       this expects a 1d array for X
    '''
    #g = 0.7964
    B = np.matrix([[.5*cos(th), .5*cos(th)],[.5*sin(th), .5*sin(th)],[1.0/(2.0*r0), -1.0/(2.0*r0)]])
    return B

def PseudoBInv(th,ro):
    Binv = np.matrix([[cos(th), sin(th),ro],[cos(th), sin(th),-ro]])
    return Binv
    
def A(dt):
    Ia = np.mat(np.eye(3))
    #Dt = dt*Ia
    #Za = np.mat(np.zeros((3,3)))
    #A = np.bmat([[Ia,Dt],[Za,Za]])
    return Ia



def DelayModel(speed):
    return 0.0;
    if speed < 17.5: return 2.0
   
    Mslope = -0.021
    MIntercept = 1.377
    delay = Mslope*speed+MIntercept
    return delay


def MotorGainAndOffset():
    #G = np.diag([0.855,0.7337])
    #V = np.mat([[-1.8889],[-0.6113]])
    G = np.diag([1.0,1.0])
    V = np.mat([[0.0],[0.0]])
    return G,V