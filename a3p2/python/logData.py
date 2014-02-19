import time
from Scribbler2 import *

# Connect to the scribbler
# Set timeout to 0 to read instantly, non-blocking

fname = "log-%d.txt" % time.time()

s = Scribbler2('COM4',fname)

# Set timeout to zero
print 'Connected!'
s.setIRPower(140)
s.setForwardness(1)
# Create a list of commands
commands = []
# Command is a list [cmd, leftMotor, rightMotor, time]
# Setting motors to 200 will drive 
# forward with the fluke facing forward
commands.append([100, 100, 12])
commands.append([-100, 100, 1.7])
commands.append([100,100, 5])
commands.append([100,-100, 1.7])
commands.append([100,100,15])
commands.append([0,0,1])
print ("Start!")
for c in commands:
  start = time.time()
  s.setMotors(c[0],c[1])
  while (time.time() - start < c[2]):
    s.getObstacle(1)
    time.sleep(0.1) # Read sensors at 1Hz



