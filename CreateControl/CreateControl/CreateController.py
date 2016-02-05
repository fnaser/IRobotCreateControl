from threading import Thread, Lock
from CreateStateEstimator import *
from CreateInterface import *
from CreateModel import *


class CreateController(Thread):
    def __init__(self,CRC,stateholder,Xks,ro,dt,Q,R):
        Thread.__init__(self)
        self.CRC = CRC
        self.dt = dt
        self.CRC.start()
        self.Xks = Xks
        self.offset = np.matrix([0,0,0])
        self.Uos = TrajToUko(Xks,ro,dt)
        self.Ks = TVLQR(self.Xks, self.Uos, dt, ro, Q, R)
        self.index = 0

    def run(self):
        while True:
            time.sleep(self.dt)
            
            # get the current State
            X = np.matrix(self.holder.getState())

            
            if(index ==0):
                # for first time step set offset and start movement
                self.offset = X-self.Xks[index]
                self.CRC.directDrive(self.Uos[index],self.Uos[index])
            else:
                #compenstate for offset between Vicon and planned coordinates
                X = X-self.offset
                #calculate the diffrence from desired state
                DX = X- self.Xks[index]
                # Generate the correction Term
                Uc = Ks[index-1].dot(Dx)
                # Make the new speed command
                U = self.Uos[index] + Uc
                # run it
                self.CRC.directDrive(U[0],U[1])
            index +=1