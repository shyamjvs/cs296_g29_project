import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as myplot 
import matplotlib.cbook as cbook
from pylab import *
import os
if not os.path.exists('../data'):
    os.makedirs('../data')

### 1) plot

f = open('../data/g29_lab09data_01.csv', 'r')

iter_list = []
step1 = []
loop1 = []
num_iter = 100
num_rerun = 30
for i in range(num_iter):
	step_total = 0
	loop_total = 0
	iter_list.append(i + 1)
	for j in range(num_rerun):
		line = f.readline()
		step_total = step_total + float(line.split()[2][:-1])
		loop_total = loop_total + float(line.split()[6])
	step_total = step_total/num_rerun
	loop_total = loop_total/num_rerun
	step1.append(step_total)
	loop1.append(loop_total)
fig, plot1 = myplot.subplots()
plot1.bar(iter_list, step1, 0.01, color = 'k', label = 'Avg. Step Time')
plot1.plot(iter_list, loop1, 'b-', label = 'Avg. Loop Time')
plot1.set_title('Graphs of averaged step time and averaged loop time vs iteration value')    
plot1.set_xlabel('Iteration value')
plot1.set_ylabel('Average time(ms)')
plot1.legend(loc = 'best')
fig.savefig('../data/g29_lab09_plot01.png')

### 2) plot

f = open('../data/g29_lab09data_01.csv', 'r')

step2 = step1
col2 = []
vel2 = []
pos2 = []
sum2 = []
for i in range(num_iter):
	total1 = 0
	total2 = 0
	total3 = 0
	for j in range(num_rerun):
		line = f.readline()
		total1 = total1 + float(line.split()[3][:-1])
		total2 = total2 + float(line.split()[4][:-1])
		total3 = total3 + float(line.split()[5][:-1])
	total1 = total1/num_rerun
	total2 = total2/num_rerun
	total3 = total3/num_rerun
	col2.append(total1)
	vel2.append(total2)
	pos2.append(total3)
	sum2.append(total1 + total2 + total3)

fig = myplot.figure()
plot2 = fig.add_subplot(111)
plot2.set_title('Graphs of avg.times of various parameters vs iteration value')    
plot2.set_xlabel('Iteration number')
plot2.set_ylabel('Average Times(ms)')
plot2.plot(iter_list, step2, '-', c = 'r', label = 'Avg. step time')
plot2.plot(iter_list, col2, 'b-', label = 'Avg. collision time')
plot2.plot(iter_list, vel2, 'g-', label = 'Avg. velocity time')
plot2.plot(iter_list, pos2, 'm-', label = 'Avg. position time')
plot2.plot(iter_list, sum2, 'k-', label = 'Sum of avg. col, vel, pos Time')
plot2.legend(loc = 'best')
fig.savefig('../data/g29_lab09_plot02.png')

### 3) plot

yerror = [iter_list/5 for iter_list in step1]
fig = myplot.figure()
plot3 = fig.add_subplot(111)
plot3.set_title('Line graph of avg. step time vs iteration value with error-bars')    
plot3.set_xlabel('Iteration value')
plot3.set_ylabel('Average Time(ms)')
plot3.errorbar(iter_list, step1, yerror, c = 'b', label = 'Step Time')
plot3.legend(loc = 'best')
fig.savefig('../data/g29_lab09_plot03.png')

### 4) plot

f=open('../data/g29_lab09data_01.csv','r')

step4 = []
roll_num = 52
for i in range((roll_num - 1) * num_rerun):
	f.readline()
for j in range((roll_num - 1) * num_rerun, roll_num * num_rerun):
	line=f.readline()
	step4.append(float(line.split()[2][:-1]))

counts = []
min_step = min(step4)
max_step = max(step4)
bins = 15
for i in range(bins):
	counts.append(0)
x2 = np.linspace(min_step,max_step,bins+1)
for i in range(len(step4)):
	start_value = min_step
	for j in range(bins - 1):
		if(step4[i] >= x2[j] and step4[i] < x2[j + 1]) :
			counts[j] += 1
x2 = x2[0:len(x2) - 1]
counts[len(counts) - 1] += 1
for i in range(1, len(counts)):
	counts[i] += counts[i-1]
fig, plot4 = myplot.subplots()
plot4.hist(step4, bins, histtype='bar', color='r', label="Frequency")
plot4.plot(x2, counts, 'r', label='Cumulative Frequency')
plot4.set_title("Histogram and cumulative frequency curve of step times(iteration value=80)")
plot4.set_xlabel('Step Time')
plot4.set_ylabel('Frequency')
plot4.legend(loc = 'best')
fig.savefig('../data/g29_lab09_plot04.png')

### 5) plot

f = open('../data/g29_lab09data_random.csv', 'r')

step5 = []
random_num = 15
for i in range(num_iter):
	total = 0
	for j in range(random_num):
		line = f.readline()
		#print(line.split()[2][:-1])
		total = total + float(line.split()[2][:-1])
	total = total/random_num
	step5.append(total)
line1 = polyfit(iter_list, step5, 1)
line2 = polyfit(iter_list, step1, 1)
fit_fn1 = poly1d(line1)
fit_fn2 = poly1d(line2)
fig = myplot.figure()
plot5 = fig.add_subplot(111)
plot5.set_title('Graphs of averaged step time and averaged loop time cs iteration value')    
plot5.set_xlabel('Iteration value')
plot5.set_ylabel('Average Time(ms)')
plot5.plot(iter_list, step5, 'r-', iter_list, fit_fn1(iter_list), label = 'Best fit line for random data')
plot5.plot(iter_list, step1, 'm-', iter_list, fit_fn2(iter_list), label = 'Best fit line for all data data')
plot5.legend(loc = 'best')
fig.savefig('../data/g29_lab09_plot05.png')
