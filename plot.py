#!/usr/bin/python3
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

file = open("remote_log", 'r')
rawdata = [str(i) for i in file]
file.close()

time = []
roll = []
setRoll = []
speed = []
setSpeed = []
zeroTime = float(rawdata[0].split(" ")[0]) + float(rawdata[0].split(" ")[1])/1000000000 + 19
for i in rawdata:
	data = i.split(" ")
	now = float(data[0]) + float(data[1])/1000000000 - zeroTime
	if (now > 0):
		time.append(now)
		roll.append(float(data[2]))
		setRoll.append(float(data[3]))
		speed.append(float(data[4]))
		setSpeed.append(float(data[5]))

fig = plt.figure()
red = mpatches.Patch(color = 'red', label = 'Set value')
blue = mpatches.Patch(color = 'blue', label = 'Real value')
plt.subplots_adjust(hspace = 0.3)

ax = fig.add_subplot(211)
plt.ylabel('roll')
plt.xlabel('time')
ax.plot(time, roll)
ax.plot(time, setRoll, 'r')
plt.legend(loc = 0, handles=[red, blue])

ax = fig.add_subplot(212)
plt.ylabel('speed')
plt.xlabel('time')
ax.plot(time, speed)
ax.plot(time, setSpeed, 'r')
plt.legend(loc = 0, handles=[red, blue])

plt.show()
