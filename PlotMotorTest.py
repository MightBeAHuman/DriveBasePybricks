from matplotlib import pyplot as plt
import sys

fig, axs = plt.subplots(2, 2)

currents = []
voltages = []
angles = []
times = []
speed = 0
wait = 0
with open(sys.argv[1], "r") as f:
    lines = f.readlines()
    speed = int(lines[1].strip().split(" ")[-1].strip())
    wait = int(lines[2].strip().split(" ")[-1].strip())
    c = 0
    for line in lines[3:]:
        c += 1
        line = line.strip()
        if line == "":
            c = 0
            continue
        if c == 4: voltages.append(int(line.split(" ")[-1].strip()))
        if c == 5: currents.append(int(line.split(" ")[-1].strip()))
        if c == 7: times.append(int(line.split(" ")[-1].strip()))
        if c == 8: angles.append(int(line.split(" ")[-1].strip()))


ax1 = axs[0][0]
ax2 = axs[0][1]
ax3 = axs[1][0]
ax4 = axs[1][1]
runs = list(range(len(currents)))

fig.suptitle("Speed: " + str(speed))
fig.text(0.5, 0.94, 'Time between runs in ms: ' + str(3000), ha='center')

ax1.set_xlabel("Run No.")
ax1.set_ylabel("Hub Voltage in mV")
ax1.plot(runs, voltages, "b-")

ax2.set_xlabel("Run No.")
ax2.set_ylabel("Hub Current in mA")
ax2.plot(runs, currents, "b-")

ax3.set_xlabel("Run No.")
ax3.set_ylabel("Motor Angles in degrees")
ax3.plot(runs, angles, "b-")

ax4.set_xlabel("Run No.")
ax4.set_ylabel("Time in ms")
ax4.set_ylim(min(times) - 10, max(times) + 10)
ax4.plot(runs, times, "b-")

plt.show()
