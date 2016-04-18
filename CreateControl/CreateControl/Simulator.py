from threading import Thread, Lock
from CreateStateEstimator import *
from CreateInterface import *
from CreateModel import *
from CreateController import *
from TrajectoryTests import *
import sys

from plotRun import plotCSVRun

class CreateSimulator(Thread):
    def __init__(self,CRC_sim,stateholder,XKs,ro,dt,Q,speedup = 10,timelock=None):
        Thread.__init__(self)
        self.CRC = CRC_sim
        self.holder = stateholder
        self.ro =ro
        self.dt = dt
        self.Q = Q
        self.Xks = XKs
        self.t0 = int(time.time()*1000)
        self.index = 0
        self.speedup=speedup
        self.ticktock = timelock
    
    def run(self):
        X_k = np.mat(np.zeros( (6,1) ) )
        print "start sim"
        while True and self.index<len(self.Xks):
            time.sleep(self.dt/self.speedup)
            U = self.CRC.LastU()
            if self.index == 0:
                Conf = self.holder.GetConfig()
                Vconf = np.mat(np.zeros((3,1)))
                X_k = np.bmat([ [Conf],[Vconf]] )

            #print "Sim: ", Conf.shape
            Conf = self.holder.GetConfig()
            #if (self.index==0): Conf[0,0]+=20.0
            n = [0.1,0.1,2*pi/360,0.0001,0.0001,0.0001]
            W = np.matrix([np.random.normal(0,n[i],1) for i in range(0,6)] )


            K1 = B(Conf[2,0],self.ro).dot(U)
            K2 = B(Conf[2,0]+self.dt*0.5*K1[2,0],self.ro).dot(U)
            K3 = B(Conf[2,0]+self.dt*0.5*K2[2,0],self.ro).dot(U)
            K4 = B(Conf[2,0]+self.dt*K3[2,0],self.ro).dot(U)


            #    B returns a 6x1 but X_k is 3x1
            X_k_p1 = A(self.dt).dot(X_k) + self.dt/6*(K1+2*K2+2*K3+K4)# +W
            #print A(self.dt)
            #print X_k_p1[0:3]
            X_k_p1[2,0] = X_k_p1[2,0]%(2.0*pi)
            X_k = X_k_p1
            print "Sim:",self.index
            
            t = self.t0+1000*self.index

            step = False
            if self.ticktock == None: step=True
            elif self.ticktock.simTick(): 
                step = True
                self.ticktock.setSimI(self.index+1)

            if step:
                self.holder.setState(X_k_p1[0:3],t)
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
    channel = 'VICON_create8'
    r_wheel = 125#mm
    dt = 1.0/5.0
    r_circle = 400#mm
    speed = 60 #64


    '''Q should be 1/distance deviation ^2
    R should be 1/ speed deviation^2
    '''
    #Q = np.eye(6)
    dist = 30.0 #mm
    ang = 1.0 # radians
    #Q = Q*(1.0/(dist*dist))
    #Q[2,2]= 1.0/(ang*ang)
    speed_deviation = 5 #mm/s
    angular_rate_deviation  = 20.0/125.0 # radians / seconds
    Q = np.diag([1.0/(dist*dist),1.0/(dist*dist),1.0/(ang*ang),1/(speed_deviation*speed_deviation),1/(speed_deviation*speed_deviation),1.0/(angular_rate_deviation* angular_rate_deviation)])

    R = np.eye(2)
    command_variation = 20.0
    R = np.diag([1/( command_variation * command_variation ), 1/( command_variation * command_variation )] )



    Xks = circle(r_circle,dt,speed)
    lock = Lock()

    start = np.matrix(Xks[0][0:3]).transpose()

    print start

    speedup = 10.0

    sh = StateHolder(lock,start)
    timelock = TickTock()

    CRC =CRC_Sim()
    CC = CreateController(CRC,sh,Xks,r_wheel,dt,Q,R,speedup,timelock)
    VSim = CreateSimulator(CRC,sh,Xks,r_wheel,dt,Q,speedup,timelock)

    VSim.start()
    #time.sleep(0.005)
    CC.start()

    CC.join()
    VSim.join()
    print 'Done'
    time.sleep(0.05)
    plotCSVRun()
    return 0


if __name__ == "__main__":
    sys.exit(int(main() or 0))