from threading import Thread, Lock
from CreateStateEstimator import *
from CreateInterface import *
from CreateModel import *
from CreateController import *
from TrajectoryTests import *
import sys
from plotRun import *

class CreateSimulator(Thread):
    def __init__(self,CRC_sim,stateholder,XKs,ro,dt,Q):
        Thread.__init__(self)
        self.CRC = CRC_sim
        self.holder = stateholder
        self.ro =ro
        self.dt = dt
        self.Q = Q
        self.Xks = XKs
        self.t0 = int(time.time()*1000)
        self.index = 0
    
    def run(self):
        while True and self.index<len(self.Xks):

            time.sleep(self.dt/10)
            U = self.CRC.LastU()
            X_k = self.holder.getState()
            if (self.index==0): X_k[0,0]+=20.0
            n = [0.1,0.1,2*pi/360]
            W = np.matrix([np.random.normal(0,n[i],1) for i in range(0,3)] )


            K1 = B(X_k[2,0],self.ro).dot(U)
            K2 = B(X_k[2,0]+self.dt*0.5*K1[2,0],self.ro).dot(U)
            K3 = B(X_k[2,0]+self.dt*0.5*K2[2,0],self.ro).dot(U)
            K4 = B(X_k[2,0]+self.dt*K3[2,0],self.ro).dot(U)
            X_k_p1 = X_k + self.dt/6*(K1+2*K2+2*K3+K4) #+W

            X_k_p1[2,0] = X_k_p1[2,0]%(2.0*pi)
            print "Sim:",self.index
            
            t = self.t0+1000*self.index
            self.holder.setState(X_k_p1,t)
            self.index+=1

class CRC_Sim:
    def __init__(self):
        self.Us=[]

    def start(self):
        pass

    def stop(self):
        pass

    def directDrive(self,V1,V2):
        V1= max(min(V1,500),-500)
        V2= max(min(V2,500),-500)
        self.Us.append(np.matrix([V1,V2]).transpose())
    
    def LastU(self):
        if len(self.Us):
            return self.Us[-1]
        return np.matrix([0,0]).transpose()

def main():
    channel = 'VICON_sawbot'
    r_wheel = 125#mm
    dt = 1.0/5.0
    r_circle = 610#mm
    speed = 64


    '''Q should be 1/distance deviation ^2
    R should be 1/ speed deviation^2
    '''
    Q = np.eye(3)
    dist = 1.0
    rad = 1.0
    speed_dev = 1.0

    Q = Q*(1.0/(dist*dist))
    Q[2,2]= 1.0/(rad*rad)
    R = np.eye(2)
    R = R*(1.0/(speed_dev*speed_dev))

    Xks = circle(r_circle,dt,speed)
    lock = Lock()
    sh = StateHolder(lock,np.matrix(Xks[0]).transpose())

    CRC =CRC_Sim()
    CC = CreateController(CRC,sh,Xks,r_wheel,dt,Q,R)
    VSim = CreateSimulator(CRC,sh,Xks,r_wheel,dt,Q)

    VSim.start()
    time.sleep(0.05)
    CC.start()

    CC.join()
    VSim.join()
    print 'Done'
    plotCSVRun()


if __name__ == "__main__":
    sys.exit(int(main() or 0))