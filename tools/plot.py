#!/usr/bin/python

# -------------------------------------------------------------------
# A script that plots data from the Arduino serial log
# 
# David Vella, June 2020
# -------------------------------------------------------------------

import matplotlib.pyplot as plt

skips = 0

t = []
x = []
y = []
z = []

with open('serial.log', 'r') as f:
    for _i in range(skips):
        f.readline()

    for line in f:
        T,X,Y,Z = line.split()

        t.append(float(T))
        x.append(float(X))
        y.append(float(Y))
        z.append(float(Z))


plt.plot(t,x,label='roll')
plt.plot(t,y,label='pitch')
plt.plot(t,z,label='yaw')

ax = plt.gca()
ax.set_ylim([-1,1])
ax.legend()

plt.title('Gyro Angle')
plt.xlabel('time (sec)')
plt.ylabel('angle (deg)')

plt.show()