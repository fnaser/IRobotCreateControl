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


def main():
    r=254.0 # 10 in 254cm 
    dt=1.0
    speed = 26.5988178 #mm / s
    xs = circle(r,dt,speed)
    fig,ax=pl.subplots()
    X = np.array(xs)

    plt.figure(1)
    plt.subplot(111)
    plt.plot(X[:,0],X[:,1],'bx-')
    plt.axis('equal')




    r_wheel = 125
    dt = 1.0/5.0
    UKs = TrajToUko(xs,r_wheel,dt)
    plt.figure(2)
    U = np.array(UKs)
    plt.plot(U[:,0])
    plt.plot(U[:,1])

    plt.show()
    return 0




if __name__ == "__main__":
    sys.exit(int(main() or 0))



