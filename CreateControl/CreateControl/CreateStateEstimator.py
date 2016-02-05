from vicon import body_t
from math import *
from threading import Thread, Lock
import time
import lcm
import sys

class StateHolder():
    def __init__(self,start,lock):
        self.state = start
        self.lock = lock
    def setState(state):
        self.lock.acquire()
        self.state = state
        self.lock.release()
    def getState(state):
        return self.state

def vicon_handler(holder,channel,data):
    msg = body_t.decode(data)
    position = msg.trans
    s,x,y,z = msg.quat
    a = 2*atan2(z,s)
    state = [position[0],position[1],a]
    holder.setState(state)

class ViconInterface(Thread):
    def __init__(self,channel,state_holder):
        self.channel = channel
        Thread.__init__(self)
        self.lc = lcm.LCM()
        handler = lambda x,y: vicon_handler(state_holder,x,y)
        self.subscription = lc.subscribe(channel,handler)

    def run(self):
        while True:
            lc.handle()


class ViconTester(Thread):
    def __init__(self,state_holder,rate):
        self.holder = state_holder
        self.rate=rate
    def run(self):
        while True:
            time.sleep(round(1.0/self.rate)*1000)
            print "rate:%0.2f"%rate, self.holder.getState()


def main():
    #http://www.tutorialspoint.com/python/python_multithreading.htm
    #http://www.toptal.com/python/beginners-guide-to-concurrency-and-parallelism-in-python
    #http://lcm.googlecode.com/svn/www/reference/lcm/tut_python.html
    channel = 'Vicon_Create8'
    locker = Lock()

    sh = StateHolder([0,0,0],locker)

    VI = ViconInterface(channel,sh)
    VT1 = ViconTester(sh,10)
    VT2 = ViconTester(sh,1)
    VI.start()
    VT1.start()
    VT2.start()

    VI.join()
    VT1.join()
    VT2.join()
    print "Exiting Main Thread"


if __name__ == "__main__":
    sys.exit(int(main() or 0))