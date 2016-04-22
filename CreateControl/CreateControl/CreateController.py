from threading import Thread, Lock
from CreateStateEstimator import *
from CreateInterface import *
from CreateModel import *
from TrajectoryTests import *
import scipy
from scipy.optimize import minimize
import sys

import csv

from math import fabs

class TickTock():
    def __init__(self):
        self.SimI=0
        self.ConI=0
        self.lock = Lock()
    def setSimI(self,I):
        #self.lock.acquire()
        self.SimI = I
        #self.lock.release()
    def setConI(self,I):
        #self.lock.acquire()
        self.ConI = I
        #self.lock.release()
	#print "set: ",state
    def simTick(self):
        return self.ConI>=self.SimI
    def conTick(self):
        return self.ConI<=self.SimI

def dBdtheta(th):
    B = np.matrix([[-0.5*sin(th), -0.5*sin(th)],
                   [ 0.5*cos(th),  0.5*cos(th)],
                   [ 0,            0]])
    return B



def makeSuperQ(Q,T):
    diag=[]
    for i in range(0,T):
            diag.append(np.zeros((2,2)))
            diag.append(Q)
    qbar = scipy.sparse.block_diag(diag).todense()
    return qbar

def obj(Xbar,Xtraj,Qbar):
        DX = (Xbar.reshape((Xbar.size,1))-Xtraj)
        J = 0.5*DX.T.dot(Qbar).dot(DX)
        return J 

def jacobian(Xbar,Xtraj,Qbar):
    DX = (Xbar-Xtraj)
    return DX.T.dot(Qbar)
    
def dynamicConstraint(Xbar,Xnow,dt,ro,T):
    G,V = MotorGainAndOffset()
    C = np.zeros((3*(T),1))
    U1 = Xbar[0:2].reshape((2,1))
    X2 = Xbar[2:5].reshape((3,1))
    Umod = G.dot(U1)+V
    C[0:3] = X2-Xnow -  dt*B(Xnow[2],ro).dot(Umod)

    for i in range(1,T):
        Xkm1 = Xbar[5*(i-1)+2    : 5*(i-1)+2+3].reshape((3,1))
        Ukm1 = Xbar[5*(i-1)+3+2  : 5*(i-1)+4+3].reshape((2,1))
        Xk =   Xbar[5*(i ) +2    : 5*(i  )+2+3].reshape((3,1))
        Umod = G.dot(Ukm1)+V
        diff = Xk - Xkm1 - dt*B(Xkm1[2],ro).dot(Umod)
        C[3*i:3*i+3] = diff
    return C

def subBlock(Xbar,dt,ro,i):
    x1 = Xbar[5*(i)+2:5*(i)+2+3]
    U1 = Xbar[5*(i)+3+2:5*(i)+4+3]
    

    G,V = MotorGainAndOffset()
    Umod = G.dot(U1)+V
    quack = dt*dbdtheta(x1(2)).dot(Umod)
    crack = np.zeros((3,3))
    crack[:,2] = quack
    Bk = B(x1[2])
    
    dC1dx1 = -np.eye(3) - crack
    dC1du1 = -dt*Bk.dot(G)
    dC1dx2 = np.eye(3)
    return np.bmat([dC1dx1,dC1du1,dC1dx2])

def dynamicJacobian(Xbar,Xnow,dt,ro,T):
    Cj = np.zereos((3*(T-1),5*T))

    Bk = B(Xnow[2])
    G,V = MotorGainAndOffset()
    U1 = Xbar[0:3]
    Umod = G.dot(U1)+V
    dC1du1 = -dt*Bk.dot(G)
    dC1dx2 = np.eye(3)
    
    Cj[0:3,0:5]=np.bmat([dC1du1,dC1dx2])

    for i in range(1,T):
        block = subBlock(Xbar,dt,ro,i)
        Cj[3*i:3*i+3,5*i-3:5*i+6] = block
    return Cj

def bounds(T,umax):
    b = []
    for i in range(0,T):
        b.append((None,None))
        b.append((None,None))
        b.append((None,None))
        b.append((-umax,umax))
        b.append((-umax,umax))
    return b


def xtrajMaker(Xks,Uks,T,index):
    xtraj = np.zeros((5*T,1))
    for i in range(0,T):
        xtraj[0+5*i:2+5*i] = Uks[index+i].transpose()
        xtraj[2+5*i:5+5*i] = Xks[index+1+i].reshape((3,1))#.transpose()
    return xtraj

class CreateController(Thread):
    def __init__(self,CRC,stateholder,Xks,ro,dt,Q,R,T,delay=0,maxU=100,speedup = 1,ticktoc = None):
        Thread.__init__(self)
        self.speedup = speedup
        self.ticktock = ticktoc
        self.delay = delay
        self.CRC = CRC
        self.holder = stateholder
        self.dt = dt
        self.T  =T
        self.Q = Q
        self.ro = ro
        self.bounds = bounds(T,50)
        self.CRC.start()
        self.Xks = Xks
        self.offset = np.matrix([0,0,0]).transpose()
        self.Uos = TrajToUko(Xks,ro,dt)
        self.Ks = TVLQR(self.Xks, self.Uos, dt, ro, Q, R)
        self.maxU = maxU
        self.index = 0
        self.csvFile = open("Run.csv",'wb')
        self.writer = csv.writer(self.csvFile)
        #row = [s[3],self.Xks[index],X,U]
        row=['Time','X_target','Y_target','Angle_target','X_actual','Y_actual','Angle_actual','DX angle','U[0]','U[1]','Uc[0]','Uc[1]']
        self.writer.writerow(row)




        
    def transform(self,X):
        
        th = -self.offset[2,0]
        Rv = np.matrix([[cos(th), -sin(th) ,0],
                       [sin(th), cos(th)  ,0],
                       [0,0,1]])
        #Zv = np.mat(np.zeros((3,3)))
        #SRv = np.bmat([[Rv,Zv],[Zv,Rv]])

        th = self.Xks[0][2]
        Rp = np.matrix([[cos(th), -sin(th) ,0],
                       [sin(th), cos(th)  ,0],
                       [0,0,1]]) 

        #SRp = np.bmat([[Rp,Zv],[Zv,Rp]])

        return Rp.dot(Rv.dot(X-self.offset))+(np.matrix(self.Xks[0][0:3]).transpose())

    def run(self):
        ndelay = int(self.delay/self.dt)
        Qbar = makeSuperQ(self.Q,self.T)
        while True and self.index<( len(self.Uos)+ndelay):

            time.sleep(self.dt/self.speedup)





            # get the current State
            X_m = self.holder.GetConfig()
            t = self.holder.getTime()

            #print "state %0.3f,%0.3f,%0.3f"%(s[0],s[1],s[2]) 
            #X = np.matrix([s[0],s[1],s[2]])
            index = self.index
            if(self.index ==0):
                # for first time step set offset and start movement
                self.offset = X_m
                print 'offset:',  self.offset


            X_m = self.transform(X_m)
            Xtraj = xtrajMaker(self.Xks,self.Uos,self.T,index)
            

            constrains = ({'type':'eq',
               'fun':lambda x: dynamicConstraint(x,X_m,self.dt,self.ro,self.T),
               'jac': lambda x: jacobian(x,Xtraj,Qbar)})


            #Xguess
            targetobj = lambda x: obj(x,Xtraj,Qbar)
            XStar = minimize(targetobj,Xtraj,method='SLSQP',
                                bounds = self.bounds,
                                constraints = constrains,
                                jac = jacobian)
            U = XStar[0:3]


            #look out for theta wrap around


            if self.index>ndelay:
                index = self.index-ndelay
            else:
                index = 0
                
            Xk = np.matrix(self.Xks[index]).transpose()
            DX = X_m- Xk
            DX[2,0] = minAngleDif(X_m[2,0],self.Xks[index][2])
            Uc = np.array(self.Uos[self.index]).transpose()


            step = False
            if self.ticktock == None: step=True
            elif self.ticktock.conTick(): 
                step = True
                self.ticktock.setConI(self.index+1)

            if step:
                self.CRC.directDrive(U[0,0],U[1,0])

                # add to log
                row = [t]+[Xk[0,0], Xk[1,0],  Xk[2,0] ]+[X_m[0,0], X_m[1,0],  X_m[2,0] ]+[DX[2,0]]+[U[0,0],U[1,0]]+[Uc[0,0],Uc[1,0]]
                print "I:",self.index

                self.writer.writerow(row)

                self.index +=1

        self.CRC.stop()
        self.csvFile.close()
        print "closed"

def main():
    channel = 'VICON_sawbot'
    r_wheel = 125#mm
    dt = 1.0/5.0

    r_circle = 300#mm
    speed = 20 #64


    '''Q should be 1/distance deviation ^2
    R should be 1/ speed deviation^2
    '''

    dist = 20.0 #mm
    ang = 1.0 # radians

    Q = np.diag([1.0/(dist*dist),
                 1.0/(dist*dist),
                 1.0/(ang*ang)])


    R = np.eye(2)
    command_variation = 10.0
    R = np.diag([1/( command_variation * command_variation ),
                 1/( command_variation * command_variation )] )



    Xks = circle(r_circle,dt,speed)
    delay =  DelayModel(speed)
    maxU = 15.0


    lock = Lock()

    start = np.matrix(Xks[0][0:3]).transpose()

    print start
    sh = StateHolder(lock,np.matrix([0,0,0]).transpose())

    VI = ViconInterface(channel,sh)

    CRC = CreateRobotCmd('/dev/ttyUSB0',Create_OpMode.Full,Create_DriveMode.Direct)
    CC = CreateController(CRC,sh,Xks,r_wheel,dt,Q,R,delay,maxU)


    
    VI.start()
    time.sleep(0.05)
    CC.start()

    CC.join()
    #VI.join()

    print "Done"
    plotCSVRun()

if __name__ == "__main__":
    sys.exit(int(main() or 0))
