from enum import IntEnum
import serial
import struct
from math import *
import sys
import time

DEBUG = True

class Create_OpMode(IntEnum):
    Passive = 130
    Safe = 131
    Full = 132

class Create_DriveMode(IntEnum):
    Drive = 137
    Direct = 145

class Create_Commands(IntEnum):
    Start = 128

def minMax(min_val,max_val,val):
    return max(min_val,min(val,max_val))

class CreateRobotCmd(object):
    def __init__(self,port,OpMode,DriveMode):
        # These are hard limits in the create which are hard coded in
        self.rmin = -2000.0
        self.rmax = 2000.0
        self.vmin = -500.0
        self.vmax = 500.0
        # opp mode is either Pasisve, safe, or full. Always use full 
        self.opmode = OpMode
        #information on drive mode is bellow and in documentation
        self.drivemode = DriveMode
        #Serial port baud is currently hard coded
        if DEBUG:
            self.port = serial.Serial()
        else:
            self.port = serial.Serial(port,57600,parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
    def close(self):
        if not DEBUG:
            self.port.close()
        else:
            print 'close called'

    def start(self):
        self._writeCommand(Create_Commands.Start)
        self._writeCommand(self.opmode)

    def _writeCommand(self,cmd):
        '''the input type should be a string or int,
           int is converted to a char string,
           strings are passed through'''
        if (type(cmd) == type(Create_Commands.Start) or 
            type(cmd) == type(Create_OpMode.Full) or 
            type(cmd) == type(Create_OpMode) ):
            cmd = int(cmd)

        if type(cmd) == int:
            cmd = str(chr(cmd))
        elif type(cmd) == type(None):
            print "huh"
            return
        #elif type(cmd) != type('str'):
        #    cmd = '%i'%int(cmd)
        nb = len(cmd)
        if not DEBUG:
            nb_written = self.port.write(cmd)
            if (nb != nb_written):print "Error only wrote %i not %i bytes"%(nb_written,nb)
        else:
            print cmd
            int_form = []
            for i in range(0,nb):
                int_form.append(int(ord(cmd[i])))
            print int_form

    def _makecmd(self,head,one,two):
        return struct.pack('B',head)+struct.pack('>h',one)+struct.pack('>h',two)

    def setDriveMode(DriveMode):
        self.drive = DriveMode
        self._writeCommand(DriveMode)

    def drive(self,Radius,Velocity):
        '''This mode uses radii and velocity for turning
            straight requires R=None
            Clockwise and Counterclockwise in place are R=0
        '''
        if (self.drivemode != Create_DriveMode.Drive): self.setDriveMode(Create_DriveMode.Drive)

        Velocity =  minMax(self.vmin,self.vmax,Velocity)

        if Radius == None:
            Radius = 32767
        elif Radius ==0 and Velocity >0:
            Radius =int('0xFFFF',0)
        elif Radius ==0 and Velocity <0:
            Radius =int('0x0001',0)
        else:
            Radius = minMax(self.rmin,self.rmax,Radius)
        cmd = self._makecmd(int(self.drivemode),int(Velocity),int(Radius))
        self._writeCommand(cmd)

    def directDrive(self,V1,V2):
        '''The direct drive mode allows control over right and left wheels directly. This uses v1 for R and V2 for L'''
        if (self.drivemode != Create_DriveMode.Direct): self.setDriveMode(Create_DriveMode.Direct)

        V1 = minMax(self.vmin,self.vmax,V1)
        V2 = minMax(self.vmin,self.vmax,V2)

        cmd = self._makecmd(int(self.drivemode),int(V1),int(V2))
        self._writeCommand(cmd)


def main():
    CRC = CreateRobotCmd('COM4',Create_OpMode.Full,Create_DriveMode.Drive)
    print CRC.port.isOpen()
    if CRC.port.isOpen() or DEBUG:
        CRC.start()
        #time.sleep(4)
        CRC.drive(500,-200)
        #time.sleep(4)
        #CRC.drive(-20,50)
        #time.sleep(4)
        return 0

if __name__ == "__main__":
    sys.exit(int(main() or 0))