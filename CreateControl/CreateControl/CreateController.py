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
        row=['Time','X_target','Y_target','Angle_target','X_actual','Y_actual','Angle_actual','DX angle','U[0]','U[1]','Uc[0]','Uc[1]']
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
        lastConf = np.array([0,0,0]).transpose()
        while True and self.index<len(self.Uos):
            time.sleep(self.dt)
            
            #print len(self.Uos),len(self.Xks)

            # get the current State
            Conf = self.holder.GetConfig()
            t = self.holder.getTime()
            
            #print "state %0.3f,%0.3f,%0.3f"%(s[0],s[1],s[2]) 
            #X = np.matrix([s[0],s[1],s[2]])
            index = self.index
            if(self.index ==0):
                # for first time step set offset and start movement
                self.offset = Conf
                print 'offset:',  self.offset

            Conf = self.transform(Conf)

            #look out for theta wrap around
            Vconf = (Conf-lastConf)/self.dt
            Vconf[2,0] = minAngleDif(Conf[2,0],lastConf[2])/self.dt
            X_m = np.concatentate((Conf,Vconf),axis=0)



            Xk = np.matrix(self.Xks[index]).transpose()
            DX = X_m- Xk
            DX[2,0] = minAngleDif(X_m[2,0],self.Xks[index][2])

            U = np.matrix(self.Uos[index]).transpose()
            Uc = np.matrix([0,0]).transpose()
            # Generate the correction Term
            if(self.index !=0):
                # Make the new speed command
                Uc = self.Ks[index].dot(DX)
                U = np.matrix(self.Uos[index]).transpose()-Uc
            # run it
            #else:
            #    U = 2.0*np.matrix(self.Uos[index]).transpose()
            self.CRC.directDrive(U[1,0],U[0,0])
            #print Uc
            #print X
            
            # Log

            
            row = [t]+[Xk[0,0], Xk[1,0],  Xk[2,0] ]+[Conf[0,0], Conf[1,0],  Conf[2,0] ]+[DX[2,0]]+[U[0,0],U[1,0]]+[Uc[0,0],Uc[1,0]]
            print "I:",index

            self.writer.writerow(row)

            self.index +=1
        self.CRC.stop()
        self.csvFile.close()
        print "closed"

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
    #print Xks[0],"\n",Xks[1]
    #print "\n"
    lock = Lock()
    sh = StateHolder(lock,np.matrix([0,0,0]).transpose())

    VI = ViconInterface(channel,sh)
    #VT1 = ViconTester(sh,10)
    #VT2 = ViconTester(sh,1)
    #VTL = ViconLogger("test1.csv",sh,10)
    CRC = CreateRobotCmd('/dev/ttyUSB0',Create_OpMode.Full,Create_DriveMode.Direct)
    CC = CreateController(CRC,sh,Xks,r_wheel,dt,Q,R)


    
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
