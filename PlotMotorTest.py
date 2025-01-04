from matplotlib import pyplot as plt
import sys

class MotorTestPlotter:
    def __init__(self, *files):
        self.currents = []
        self.voltages = []
        self.angles = []
        self.times = []
        self.speeds = []
        self.ports = []
        self.waitings = []
        self.filenames = []

        self.fig, self.axes = plt.subplots(2,2)

        for file in files: self.load_file(file)

    def load_file(self, file):
        lines = open(file, "r").readlines()
        self.speeds.append(int(lines[1].strip().split(" ")[-1].strip()))
        self.waitings.append(int(lines[2].strip().split(" ")[-1].strip()))
        self.ports.append(lines[3].strip().split(" ")[-1].strip())
        

        voltages = []
        currents = []
        times = []
        angles = []
        c = 0
        for line in lines[4:]:
            c += 1
            line = line.strip()
            if line == "": c = 0; continue
            match c:
                case 4: voltages.append(int(line.split(" ")[-1].strip()))
                case 5: currents.append(int(line.split(" ")[-1].strip()))
                case 7: times.append(int(line.split(" ")[-1].strip()))
                case 8: angles.append(abs(int(line.split(" ")[-1].strip())))
                case _: pass

        self.voltages.append(voltages)
        self.currents.append(currents)
        self.times.append(times)
        self.angles.append(angles)
        self.filenames.append(file)

    def plot(self, lines, title, subtitle = None, limits = None):
        self.fig.suptitle(title)
        if subtitle != None: self.fig.text(0.5, 0.94, subtitle, ha='center')
        axes = []
        for i in range(2):
             for j in range(2):
                 axes.append(self.axes[i][j])

        for axis in axes: axis.set_xlabel("Run No.")

        runs = list(range(max([len(x) for x in self.currents])))
        plots = []
        labels = []
        for i, line in enumerate(lines):
            labels.append(self.filenames[i])    
            plots.append(axes[0].plot(runs, self.voltages[i], line)[0])
            axes[0].set_ylabel("Hub Voltage in mV")
            axes[1].plot(runs, self.currents[i], line)
            axes[1].set_ylabel("Hub Current in mA")
            axes[2].plot(runs, self.angles[i], line)
            axes[2].set_ylabel("Motor Angles in degrees")
            axes[3].plot(runs, self.times[i], line)
            axes[3].set_ylabel("Time in ms")

        min_time = min([min(times) for times in self.times]) - 10
        max_time = max([max(times) for times in self.times]) + 10
        axes[3].set_ylim(min_time, max_time)
        self.fig.legend(plots, labels, loc = "upper right")

        plt.show()

plotter = MotorTestPlotter("gray_normal_fwd.txt", "gray_normal_bwd.txt", "gray_module_bwd.txt")
plotter.plot(["b-","g-","r-"], "Motor Tests Gray")
plotter = MotorTestPlotter("white_normal_fwd.txt", "white_normal_bwd.txt", "white_module_bwd.txt")
plotter.plot(["b-","g-","r-"], "Motor Tests White")

