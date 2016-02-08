from threading import Thread, Lock
from CreateStateEstimator import *
from CreateInterface import *
from CreateModel import *
from CreateController import *
from TrajectoryTests import *
import sys


class CreateSimulator(Thread):
    def __init__(self,CRC_sim,stateholder,ro,dt,Q):
        Thread.__init__(self)
        self.CRC = CRC_sim
        self.holder = stateholder
        self.ro =ro
        self.dt = dt
        self.Q = Q
        self.t0 = int(time.time()*1000)
        self.index = 0
    
    def run(self):
        while True:
            time.sleep(self.dt)
            U = self.CRC.LastU()
            X_k_mn1 = self.holder.getState()
            W = np.matrix([np.random.normal(0,self.Q[i,i],1) for i in range(0,3)] )
            X_k = X_k_mn1 + B(X_k_mn1,self.ro).dot(U)+W
            t = self.t0+1000*self.index
            self.holder.setState(X_k,t)
            self.index+=1

class CRC_Sim:
    def __init__(self):
        self.Us=[]

    def start(self):
        pass

    def stop(self):
        pass

    def directDrive(self,V1,V2):
        print [V1,V2]
        self.Us.append(np.matrix([V1,V2]).transpose())
    
    def LastU(self):
        return self.Us[-1]

def main():
    channel = 'VICON_create8'
    r_wheel = 125#mm
    dt = 1.0
    r_circle = 610#mm
    speed = 64


    '''Q should be 1/distance deviation ^2
    R should be 1/ speed deviation^2
    '''
    Q = np.eye(3)
    Q = Q*(1.0/1.0)
    R = np.eye(2)
    R = R*(1.0/50.0)

    Xks = circle(r_circle,dt,speed)
    lock = Lock()
    sh = StateHolder(lock,[0,0,0])

    CRC =CRC_Sim()
    CC = CreateController(CRC,sh,Xks,r_wheel,dt,Q,R)
    VSim = CreateSimulator(CRC,sh,r_wheel,dt,Q)

    VSim.start()
    CC.start()

    CC.join()
    VSim.join()
    print 'Done'


if __name__ == "__main__":
    sys.exit(int(main() or 0))