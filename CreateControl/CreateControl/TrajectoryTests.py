import numpy as np
from math import *
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import sys
import pylab as pl


def circle(r,dt,speed,delay=0):
    # mm,s,mm/s
    dist = speed*dt
    c = pi*r*2
    n = int(round(c/dist))
    Xks=[]
    for i in range(0,n+1):
        if ( i==0 and delay):
            p = int(delay/dt)+1
            
            print p
            for j in range(0,p):
                x=r
                y=0
                theta = pi/2.0
                Xks.append([x,y,theta])

        x = r*cos(2*pi/n*i)
        y = r*sin(2*pi/n*i)
        theta = (2*pi/n*i+pi/2.0)%(2*pi)
        Xks.append([x,y,theta])

    return np.array(Xks)


def main():
    r=254.0 # 10 in 254cm 
    dt=1.0
    speed = 26.5988178 #mm / s
    xs = circle(r,dt,speed)
    fig,ax=pl.subplots()
    X = np.array(xs)
    ax.plot(X[:,0],X[:,1],'bx-')
    ax.autoscale()
    ax.margins(0.1)
    ax.axis('equal')
    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    sys.exit(int(main() or 0))



