#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

arr = np.array([ 0.,  2.,  0.,  0.,  0.,  0.,  3.,  0.,  0.,  0.,  0.,  0.,  0.,
        0.,  0.,  2.,  0.,  0.,  0.,  0.,  0.,  1.,  0.,  0.,  0.,  0.,
        0.,  0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  1.,  1.,
        0.,  0.,  0.,  0.,  2.,  0.,  3.,  1.,  0.,  0.,  2.,  2.,  0.,
        0.,  0.,  0.,  0.,  0.,  0.,  0.,  1.,  1.,  0.,  0.,  0.,  0.,
        0.,  0.,  2.,  0.,  0.,  0.,  0.,  0.,  1.,  0.,  0.,  0.,  0.,
        0.,  0.,  0.,  0.,  0.,  3.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,
        0.,  0.,  1.,  0.,  0.,  0.,  1.,  2.,  2.])



labels, counts = np.unique(arr, return_counts=True)
print(labels, counts)
plt.bar(labels, counts, align='center')
plt.gca().set_xticks(labels)
plt.show()