from threading import Thread, Lock
from CreateStateEstimator import *
from CreateInterface import *
from CreateModel import *
from TrajectoryTests import *
import sys


class CreateController(Thread):
    def __init__(self,CRC,stateholder,Xks,ro,dt,Q,R):
        Thread.__init__(self)
        self.CRC = CRC
        self.holder = stateholder
        self.dt = dt
        self.CRC.start()
        self.Xks = Xks
        self.offset = np.matrix([0,0,0])
        self.Uos = TrajToUko(Xks,ro,dt)
        self.Ks = TVLQR(self.Xks, self.Uos, dt, ro, Q, R)
        self.index = 0

    def run(self):
        while True and self.index<len(self.Xks):
            time.sleep(self.dt)
            
            # get the current State
            s = self.holder.getState()
            print "state %0.3f,%0.3f,%0.3f"%(s[0],s[1],s[2]) 
            X = np.matrix([s[0],s[1],s[2]])
            index = self.index
            
            if(self.index ==0):
                # for first time step set offset and start movement
                self.offset = X-self.Xks[index]
                print "Offset: ",self.offset
                print 'Xo:',X-self.offset
                print 'DXo:',X-self.offset-self.Xks[index]
                print "Uos: ",self.Uos[index]
                self.CRC.directDrive(self.Uos[index][0],self.Uos[index][1])
            else:
                #compenstate for offset between Vicon and planned coordinates
                X = X-self.offset
                #calculate the diffrence from desired state
                DX = X- self.Xks[index]
                print "X:",X,'\t',X.shape
                print "DX:",DX,'\t',DX.shape
                print 'K:',self.Ks[index-1],'\t',self.Ks[index-1].shape
                print '\n\n'
                # Generate the correction Term
                Uc = self.Ks[index-1].dot(DX.transpose())
                # Make the new speed command
                U = np.matrix(self.Uos[index]).transpose() #+ Uc
                # run it
                print 'Uc:',Uc,'\t',Uc.shape
                print 'Uo:',self.Uos[index],'\t',Uc.shape
                print 'U',U
                self.CRC.directDrive(U[0],U[1])
            self.index +=1

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
    Q = Q*(1.0/10.0)
    R = np.eye(2)
    R = R*(1.0/50.0)


    Xks = circle(r_circle,dt,speed)
    lock = Lock()
    sh = StateHolder(lock,[0,0,0])

    VI = ViconInterface(channel,sh)
    #VT1 = ViconTester(sh,10)
    #VT2 = ViconTester(sh,1)
    VTL = ViconLogger("test1.csv",sh,10)
    CRC = CreateRobotCmd('/dev/ttyUSB0',Create_OpMode.Full,Create_DriveMode.Direct)
    CC = CreateController(CRC,sh,Xks,r_wheel,dt,Q,R)


    CC.start()
    VI.start()
    VTL.start()

    CC.join()
    VI.join()
    VTL.join()
    print "Done"

if __name__ == "__main__":
    sys.exit(int(main() or 0))
