#!/usr/bin/env python

import matplotlib.pyplot as plt
from math import sqrt

f = open('../logs/states.log', 'r')

truth = []
estimate = []

for line in f:
    l = line.split(",")
    assert len(l) == 5
    l[4] = l[4].strip('\n')
    if l[4] == 't':
        truth.append(l[1:3])
    elif l[4] == 'e':
        estimate.append(l[1:3])

for i in range(len(truth)):
    for j in range(2):
        truth[i][j] = float(truth[i][j])
        estimate[i][j] = float(estimate[i][j])

mse = []
for i in range(len(truth)):
    epx = abs(truth[i][0] - estimate[i][0]) - 1
    epy = abs(truth[i][1] - estimate[i][1]) - 1

    mse.append(sqrt(epx**2 + epy**2))


plt.plot(range(len(truth)), mse)
plt.xlabel('Time')
plt.ylabel('Distance Error')
plt.title('Distance Error')
plt.show()
f.close()
