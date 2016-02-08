from threading import Thread, Lock
from CreateStateEstimator import *
from CreateInterface import *
from CreateModel import *
from TrajectoryTests import *
import sys

import csv


class CreateController(Thread):
    def __init__(self,CRC,stateholder,Xks,ro,dt,Q,R):
        Thread.__init__(self)
        self.CRC = CRC
        self.holder = stateholder
        self.dt = dt
        self.CRC.start()
        self.Xks = Xks
        self.offset = np.matrix([0,0,0]).transpose()
        self.Uos = TrajToUko(Xks,ro,dt)
        self.Ks = TVLQR(self.Xks, self.Uos, dt, ro, Q, R)
        self.index = 0
        self.csvFile = open("Run.csv",'wb')
        self.writer = csv.writer(self.csvFile)
        #row = [s[3],self.Xks[index],X,U]
        row=['Time','X_target','Y_target','Angle_target','X_actual','Y_actual','Angle_actual','U[0]','U[1]','Uc[0]','Uc[1]']
        self.writer.writerow(row)
        

    def run(self):
        while True and self.index<len(self.Uos):
            time.sleep(self.dt/100)
            
            #print len(self.Uos),len(self.Xks)

            # get the current State
            X = self.holder.getState()
            t = self.holder.getTime()
            
            #print "state %0.3f,%0.3f,%0.3f"%(s[0],s[1],s[2]) 
            #X = np.matrix([s[0],s[1],s[2]])
            index = self.index
            if(self.index ==0):
                # for first time step set offset and start movement
                self.offset = X-np.matrix(self.Xks[index]).transpose()

            X = X-self.offset
            DX = X- np.matrix(self.Xks[index]).transpose()
            U = np.matrix(self.Uos[index]).transpose()
            Uc = np.matrix([0,0]).transpose()
            # Generate the correction Term
            if(self.index !=0):
                # Make the new speed command
                Uc = self.Ks[index-1].dot(DX)/1000.0
                U = np.matrix(self.Uos[index]).transpose()-Uc
            # run it
            self.CRC.directDrive(U[1,0],U[0,0])
            
            
            # Log
            print X[1,0]
            row = [t]+self.Xks[index].tolist()+[X[0,0], X[1,0],  X[2,0] ]+[U[0,0],U[1,0]]+[Uc[0,0],Uc[1,0]]
            print "I:",index

            self.writer.writerow(row)

            self.index +=1
        self.CRC.stop()
        self.csvFile.close()
        print "closed"

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

    VI = ViconInterface(channel,sh)
    #VT1 = ViconTester(sh,10)
    #VT2 = ViconTester(sh,1)
    #VTL = ViconLogger("test1.csv",sh,10)
    CRC = CreateRobotCmd('/dev/ttyUSB0',Create_OpMode.Full,Create_DriveMode.Direct)
    CC = CreateController(CRC,sh,Xks,r_wheel,dt,Q,R)


    CC.start()
    VI.start()
    #VTL.start()

    CC.join()
    VI.join()
    #VTL.join()
    print "Done"

if __name__ == "__main__":
    sys.exit(int(main() or 0))
