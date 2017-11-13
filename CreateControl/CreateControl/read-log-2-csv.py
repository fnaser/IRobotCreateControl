import sys
import lcm
from vicon import body_t
import csv

def writeMyRow(t, pos, color, writer):
    state = [t, pos[0]*mm, pos[1]*mm, pos[2]*mm, color]
    writer.writerow(state)

if len(sys.argv) < 2:
    sys.stderr.write("usage: read-log <logfile>\n")
    sys.exit(1)

mm = 1000.0

log = lcm.EventLog(sys.argv[1], "r")

csvFile = open(sys.argv[1] + ".csv",'wb')
writer = csv.writer(csvFile)

for event in log:
    
    if event.channel == "VICON_fn_blue":
        msg = body_t.decode(event.data)
        writeMyRow(msg.utime, msg.trans, "blue", writer)
        
    if event.channel == "VICON_fn_green":
        msg = body_t.decode(event.data)
        writeMyRow(msg.utime, msg.trans, "green", writer)
        
    if event.channel == "VICON_fn_green_big":
        msg = body_t.decode(event.data)
        writeMyRow(msg.utime, msg.trans, "green_big", writer)
        
    if event.channel == "VICON_fn_yellow":
        msg = body_t.decode(event.data)
        writeMyRow(msg.utime, msg.trans, "yellow", writer)
        
    if event.channel == "VICON_fn_yellow_big":
        msg = body_t.decode(event.data)
        writeMyRow(msg.utime, msg.trans, "yellow_big", writer)
        
    if event.channel == "VICON_fn_roomba":
        msg = body_t.decode(event.data)
        writeMyRow(msg.utime, msg.trans, "roomba", writer)

csvFile.close()