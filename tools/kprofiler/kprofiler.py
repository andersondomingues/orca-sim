import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.collections import PolyCollection
import tkinter as tk
from tkinter import filedialog

####################################################
###              REQUIREMENTS                    ###
### 1) pip, $sudo apt install python3-pip        ###
### 2) matplotlib, $pip3 install matplotlib      ###
### 3) tkinter, $sudo apt install python3-tk     ###
####################################################

### display file dialog, stores path in "file_path"
root = tk.Tk()
root.withdraw()
file_path = filedialog.askopenfilename()

data_holder = []

### parse data from file
f = open(file_path, "r")
current_line_number = 0;
for x in f:
	current_line_number = current_line_number +1;
	try:
		info = x.split()
		task_number = info[0]  #first element is task id 
		task_time   = info[-1] #last element is the time the task were released
		data_holder.append([int(task_number), int(task_time)]) #add info to a temporary list
	except:
		print("WARN: ignored line ", current_line_number)

### close the file and print gethered info
f.close()

### get max number of tasks 
num_tasks = 0
for x in data_holder:
	if x[0] > num_tasks:
		num_tasks = x[0]

print("INFO: parsed ", num_tasks, " tasks")


## prepare graphic

## << -- 
data = [    (dt.datetime(2018, 7, 17, 0, 15), dt.datetime(2018, 7, 17, 0, 30), 'sleep'),
            (dt.datetime(2018, 7, 17, 0, 30), dt.datetime(2018, 7, 17, 0, 45), 'eat'),
            (dt.datetime(2018, 7, 17, 0, 45), dt.datetime(2018, 7, 17, 1, 0), 'work'),
            (dt.datetime(2018, 7, 17, 1, 0), dt.datetime(2018, 7, 17, 1, 30), 'sleep'),
            (dt.datetime(2018, 7, 17, 1, 15), dt.datetime(2018, 7, 17, 1, 30), 'eat'), 
            (dt.datetime(2018, 7, 17, 1, 30), dt.datetime(2018, 7, 17, 1, 45), 'work')
        ]

cats = {"sleep" : 1, "eat" : 2, "work" : 3}
colormapping = {"sleep" : "C0", "eat" : "C1", "work" : "C2"}

verts = []
colors = []

for d in data:
    v =  [(mdates.date2num(d[0]), cats[d[2]]-.4),
          (mdates.date2num(d[0]), cats[d[2]]+.4),
          (mdates.date2num(d[1]), cats[d[2]]+.4),
          (mdates.date2num(d[1]), cats[d[2]]-.4),
          (mdates.date2num(d[0]), cats[d[2]]-.4)]
    verts.append(v)
    colors.append(colormapping[d[2]])

bars = PolyCollection(verts, facecolors=colors)

fig, ax = plt.subplots()

ax.add_collection(bars)
ax.autoscale()
loc = mdates.MinuteLocator(byminute=[0,15,30,45])

ax.xaxis.set_major_locator(loc)
ax.xaxis.set_major_formatter(mdates.AutoDateFormatter(loc))

ax.set_yticks([1,2,3])
ax.set_yticklabels(["sleep", "eat", "work"])

### display the plot 
plt.show() 