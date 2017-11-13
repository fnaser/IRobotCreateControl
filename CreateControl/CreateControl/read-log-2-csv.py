import sys
import lcm
from vicon import body_t
import csv
import numpy as np

def writeMyRow(t, pos, writer):
    state = [t, pos[0]*mm, pos[1]*mm, pos[2]*mm]
    writer.writerow(state)

if len(sys.argv) < 2:
    sys.stderr.write("usage: read-log <logfile>\n")
    sys.exit(1)

mm = 1000.0

csvFileName = sys.argv[1] + ".csv"
print(csvFileName)

log = lcm.EventLog(sys.argv[1], "r")

csvFile = open(csvFileName,'wb')
writer = csv.writer(csvFile)

for event in log:
    if event.channel == "VICON_fn_blue":
        msg = body_t.decode(event.data)
        writeMyRow(msg.utime, msg.trans, writer)

csvFile.close()