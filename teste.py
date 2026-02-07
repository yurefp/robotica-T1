import random

#a = [-2.96706, -3.22886, -2.3911, -3.2287, -2.0944, -4.7124]
#b = [2.96706, 1.13446, 2.84489, 3.2287, 2.0944, 4.7124]
a = [-1.5708, -3.22886]
b = [1.5708, 3.2287]
x = [0]*len(a)

for i in range(4):
    for i in range(len(a)):
        x[i] = round(random.random() * (b[i]-a[i]) + a[i], 4)
    print(x)