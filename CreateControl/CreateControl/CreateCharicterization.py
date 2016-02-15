from threading import Thread, Lock
from CreateStateEstimator import *
from CreateInterface import *
from CreateModel import *
from TrajectoryTests import *
import sys

import csv


class CreateCharicterizer(Thread):
    def __init__(self,CRC,stateholder,speeds,time_step):
        Thread.__init__(self)
        self.CRC = CRC
        self.holder = stateholder
        self.speeds = speeds
        self.time = time_step

        self.CRC.start()
        self.offset = np.matrix([0,0,0]).transpose()
        self.t0 = 0
        self.csvFile = open("Run.csv",'wb')
        self.writer = csv.writer(self.csvFile)
        #row = [s[3],self.Xks[index],X,U]
        row=['Time','U_set','X','Y','Theta']
        self.writer.writerow(row)
        
    def transform(self,X):
        
        th = -self.offset[2,0]
        Rv = np.matrix([[cos(th), -sin(th) ,0],
                       [sin(th), cos(th)  ,0],
                       [0,0,1]])
        th = self.Xks[0][2]
        Rp = np.matrix([[cos(th), -sin(th) ,0],
                       [sin(th), cos(th)  ,0],
                       [0,0,1]]) 

        return Rp.dot(Rv.dot(X-self.offset))+(np.matrix(self.Xks[0]).transpose())

    def run(self):
        first = True
        while True:
            
            X = self.holder.getState()
            t = self.holder.getTime()
            
            
            if(first):
                # for first time step set offset and start movement
                self.offset = X
                self.t0 = t
                print 'offset:',  self.offset
                first = False
            
            t = (t-self.t0)/1000000 #convert to seconds since start

            index = floor(t/self.time)

            if index > len(self.speeds) or t>(len(self.speeds)*self.time):
                self.csvFile.close()
                print "closed"
                break
            
            speed = self.speeds[index]
            self.CRC.directDrive(speed,speed)

            row = [t,speed]+[X[0,0], X[1,0],  X[2,0] ]
            self.writer.writerow(row)

def main():
    
    channel = 'VICON_create8'

    speeds = [1,10,50,100,150,200]
    time_step = 5 #s


    lock = Lock()
    sh = StateHolder(lock,np.matrix([0,0,0]).transpose())

    VI = ViconInterface(channel,sh)
    CRC = CreateRobotCmd('/dev/ttyUSB0',Create_OpMode.Full,Create_DriveMode.Direct)
    CC = CreateCharicterizer(CRC,sh,speeds,time_step)


    
    VI.start()
    time.sleep(0.05)
    CC.start()
    #VTL.start()

    CC.join()
    VI.join()
#    VTL.join()
    print "Done"

if __name__ == "__main__":
    sys.exit(int(main() or 0))
