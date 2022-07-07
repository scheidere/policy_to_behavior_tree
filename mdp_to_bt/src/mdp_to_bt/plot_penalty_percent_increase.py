#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

percent_increase_array = np.array([[ 0.13044244 , 1.69696155,  2.94707005,  6.08406222]
 ,[ 1.4972554 ,  3.18467376 , 5.86843719 , 9.57479936]
 ,[ 2.50434282 , 5.11879251 , 8.68033415, 15.81227091]
 ,[ 5.47445423,  9.58134961, 18.00047205, 30.84421966]])

fig, ax = plt.subplots(1,1)

img = ax.imshow(percent_increase_array,extent=[.5, 4.5, 0.5, 4.5],origin='lower')

x_label_list = ['1', '2', '3', '4']
y_label_list = ['1', '2', '3', '4']

ax.set_xticks([1, 2, 3, 4])
ax.set_yticks([1, 2, 3, 4])

ax.set_xticklabels(x_label_list)
ax.set_yticklabels(y_label_list)

plt.xlabel('False Negative Penalty')
plt.ylabel('False Positive Penalty')

fig.colorbar(img)
plt.show()

