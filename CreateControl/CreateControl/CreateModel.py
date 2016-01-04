from math import *

''' Since we are using a vicon system we can assume that we are using a model
X_k+1 = f(X_k,U_k)+Q
Y_k+1 = I*X_k +0   so C=I and R=0

X_k is [x,y,Theta]
U_k is [Us,Uo]

f(x,u) = [Us*cos(Theta)*dt,
          Us*sin(Theta)*dt,
          Us/L*tan(Uo)*dt]
'''


class CreateModel:
    def __init__(self,r,Phi_max,dt=1):
        ''' Creates A Model for a given radius'''
        self.R = r
        self.Phi_max = Phi_max
        self.L = r*tan(Phi_max)
        self.dt = dt
    def f(self,X_k,U_k):
        ''' Returns X_k+1 for a given U_k'''
        [x,y,Theta] = X_k
        [Us,Uo] = U_k
        return [Us*cos(Theta)*self.dt,Us*sin(Theta)*self.dt,Us/self.L*tan(Uo)*self.dt]

