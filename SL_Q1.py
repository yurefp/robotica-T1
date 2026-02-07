import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
import math

d6DH = - (0.185 + 0.461877 - 0.000361/math.tan(math.radians(33.86)))

L1 = rtb.RevoluteDH(d =0.575, a=0.175, alpha=np.pi/2, offset=0)
L2 = rtb.RevoluteDH(d=0.0, a=0.89, alpha=0, offset=0)
L3 = rtb.RevoluteDH(d=0.0, a=0.0, alpha=-np.pi/2, offset=0)
L4 = rtb.RevoluteDH(d=-0.05, a=1.035, alpha=0, offset=0)
L5 = rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, offset=-np.pi/2)
L6 = rtb.RevoluteDH(d=d6DH, a=0.5, alpha=0, offset=np.pi/2)

robot = rtb.DHRobot([L1, L2, L3, L4, L5, L6], name="Robo_2R")

q = [0, 0, 0, 0, 0, 0]
robot.plot(q, block=True)

plt.show()   # ← ISSO resolve
