import matplotlib.pyplot as plt

x = [1,2,3,4]
y = [1,2,3,4]

#bar = plt.bar(y, y, align='center')

labels = ['p: 100\nd: 94', 'p: 2\n', 'p: 3\n', 'p: 4\n']
#labels = [1,2,3,4]

p_vals = [101,100,99,98]
d_vals = [94,89, 84, 80]

# for x, y, p in zip(x, y, labels):
   # plt.text(x, y, p)

# n = [1,2,3,4,5,]
# s = [i**2 for i in n]
line = plt.bar(x,y)
plt.xlabel('Number')
plt.ylabel("Square")

for i in range(len(labels)):
    plt.annotate('p:'+str(p_vals[i])+' d:'+str(d_vals[i]), xy=(x[i],y[i]), ha='center', va='bottom')

plt.show()

