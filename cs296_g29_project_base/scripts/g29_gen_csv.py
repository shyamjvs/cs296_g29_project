import subprocess
import os
import random
if not os.path.exists('../data'):
    os.makedirs('../data')



filename = open('../data/g29_lab09data_01.csv','w')
eachline = ''
num_iter = 100
num_rerun = 30
for i in range(num_iter):
	for j in range(num_rerun):
		eachline=str(i + 1) + ', ' + str(j + 1) + ', '
		create = '../mybins/cs296_29_exe ' + str(i + 1) + ' > output.txt' # creating temporary file
		os.system(create)
		with open('output.txt') as lines:
			linenumber=0
			for line in lines:
				if linenumber == 0:
					linenumber += 1;
					continue;
				for word in line.split():
					if word[0].isdigit():
						eachline = eachline + word + ', '
		eachline = eachline[:-2]
		filename.write(eachline + '\n')
os.system('rm -r -f output.txt') # deleting temporary file



filename = open('../data/g29_lab09data_random.csv', 'w')
eachline = ''
for i in range(num_iter):
	rand_sample = sorted(random.sample(range(1, 151), 15))
	for j in rand_sample:
		eachline = str(i + 1) + ', ' + str(j + 1) + ', '
		create = '../mybins/cs296_29_exe ' + str(i + 1) + ' > output.txt' # creating temporary file
		os.system(create)
		with open('output.txt') as lines:
			for line in lines:
				for word in line.split():
					if word[0].isdigit() and not word.isdigit():
						eachline = eachline + word + ', '
		eachline = eachline[:-2]
		filename.write(eachline + '\n')
os.system('rm -r -f output.txt') # deleting temporary file
