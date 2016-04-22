from threading import Thread, Lock
from CreateStateEstimator import *
from CreateInterface import *
from CreateModel import *
from TrajectoryTests import *
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



class CreateController(Thread):
    def __init__(self,CRC,stateholder,Xks,ro,dt,Q,R,delay=0,maxU=100,speedup = 1,ticktoc = None):
        Thread.__init__(self)
        self.speedup = speedup
        self.ticktock = ticktoc
        self.delay = delay
        self.CRC = CRC
        self.holder = stateholder
        self.dt = dt
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
            

            #look out for theta wrap around


            if self.index>ndelay:
                index = self.index-ndelay
            else:
                index = 0
                
            Xk = np.matrix(self.Xks[index]).transpose()
            DX = X_m- Xk
            DX[2,0] = minAngleDif(X_m[2,0],self.Xks[index][2])

            U = np.matrix(self.Uos[index]).transpose()
            Uc = np.matrix([0,0]).transpose()
            # Generate the correction Term
            if(self.index !=0):
                # Make the new speed command
                Uc = self.Ks[index].dot(DX)

                for u in range(0,2):
                    uv = Uc[u,0]
                    if fabs(uv)>self.maxU:
                        Uc[u] = self.maxU*uv/fabs(uv)
                        print "|",uv,"| > ",self.maxU,"\n" 
                U = np.matrix(self.Uos[index]).transpose()#-Uc
                # run it


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
