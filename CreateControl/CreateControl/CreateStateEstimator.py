from vicon import body_t
from math import *
from threading import Thread, Lock
import time
import lcm
import sys

import csv

class StateHolder():
    def __init__(self,lock,start):
        self.state = start
        self.lock = lock
    def setState(self,state):
        self.lock.acquire()
        self.state = state
        self.lock.release()
	#print "set: ",state
    def getState(self):
        return self.state

def vicon_handler(holder,channel,data):
    msg = body_t.decode(data)
    t = msg.utime
    position = msg.trans
    s,x,y,z = msg.quat
    a = 2*atan2(z,s)
    state = [position[0]*1000.0,position[1]*1000.0,a,t] #[mm,mm,rad,ms since epoch]
    #print 'handeled:', state
    holder.setState(state)

class ViconInterface(Thread):
    def __init__(self,channel,state_holder):
        self.channel = channel
        Thread.__init__(self)
        self.lc = lcm.LCM()
        handler = lambda x,y: vicon_handler(state_holder,x,y)
        self.subscription = self.lc.subscribe(channel,handler)

    def run(self):
        while True:
            self.lc.handle()


class ViconTester(Thread):
    def __init__(self,state_holder,rate):
        Thread.__init__(self)
        self.holder = state_holder
        self.rate=rate
	    #print "interval: ",1.0/self.rate
    def run(self):
        while True:
            time.sleep(1.0/self.rate)
            print "rate:%0.2f"%self.rate, self.holder.getState()

class ViconLogger(Thread):
    def __init__(self,name,state_holder,rate):
        Thread.__init__(self)
        self.holder = state_holder
        self.rate=rate
        self.csvFile = open(name,'wb')
        self.writer = csv.writer(self.csvFile)

	    #print "interval: ",1.0/self.rate
    def run(self):
        while True:
            time.sleep(1.0/self.rate)
            v = self.holder.getState()
            self.writer.writerow(v)


def main():
    #http://www.tutorialspoint.com/python/python_multithreading.htm
    #http://www.toptal.com/python/beginners-guide-to-concurrency-and-parallelism-in-python
    #http://lcm.googlecode.com/svn/www/reference/lcm/tut_python.html
    channel = 'VICON_create8'
    lock = Lock()

    sh = StateHolder(lock,[0,0,0])

    VI = ViconInterface(channel,sh)
    #VT1 = ViconTester(sh,10)
    VT2 = ViconTester(sh,1)
    VI.start()
    #VT1.start()
    VT2.start()

    VI.join()
    #VT1.join()
    VT2.join()
    print "Exiting Main Thread"


if __name__ == "__main__":
    sys.exit(int(main() or 0))
