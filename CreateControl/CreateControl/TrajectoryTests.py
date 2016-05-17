import numpy as np
from math import *
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import sys
import pylab as pl
from CreateModel import *

def circle(r,dt,speed):
    # mm,s,mm/s
    dist = speed*dt
    c = pi*r*2
    n = int(round(c/dist))
    Xks=[]
    for i in range(0,n+1):
        x = r*cos(2*pi/n*i)
        y = r*sin(2*pi/n*i)
        theta = (2*pi/n*i+pi/2.0)%(2*pi)
        Xk = [x,y,theta]



        Xks.append(Xk)

    return np.array(Xks)


def straight(l,dt,speed):
    dist_per_step = speed*dt
    n = int(round(l/dist_per_step))
    Xks = []
    for i in range(0,n+1):
        theta = pi/4
        r = i*dist_per_step
        x = r*cos(theta)
        y = r*sin(theta)
        Xk = [x,y,theta]
        Xks.append(Xk)
    return np.array(Xks)
    


def loadTraj(file):
    return np.load(file)


def main():
    r=254.0 # 10 in 254cm 
    dt=1.0/5.0
    speed = 26.5988178 #mm / s
    xs = circle(r,dt,speed)

    xs = straight(100,1.0/5.0,speed)

    xs = loadTraj('../Media/Cardiod-rc300.00-spacing5.00-rcut127.00-trajs-0.npy')

    fig,ax=pl.subplots()
    X = np.array(xs)

    plt.figure(1)
    plt.subplot(111)
    plt.plot(X[:,0],X[:,1],'bx-')
    plt.axis('equal')


    

    r_wheel = 125
    UKs = TrajToUko(xs,r_wheel,dt)
    U = np.vstack(UKs)
    
    T = np.linspace(0,X.shape[0]*dt,U.shape[0])
    plt.figure(2)
    plt.subplot(211)
    plt.plot(T,U[:,0],'bo-')
    plt.plot(T,U[:,1],'gx-')
    plt.subplot(212)
    T = np.linspace(0,X.shape[0]*dt,X.shape[0])
    plt.plot(T,X[:,0],'go-')
    plt.plot(T,X[:,1],'bx-')
    #print U

    plt.show()
    return 0




if __name__ == "__main__":
    sys.exit(int(main() or 0))



