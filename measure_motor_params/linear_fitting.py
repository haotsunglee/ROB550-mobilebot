from scipy import stats
# import scipy
import numpy as np
import matplotlib.pyplot as plt

with open('./data_R.txt') as f:
    lines = f.readlines()

vel = []
pwm = []
for line in lines:
    info = line.split(',')
    pwm.append(float(info[0]))
    vel.append(float(info[1].strip('\n')))

pwm = np.array(pwm)
vel = np.array(vel)

# z = np.polyfit(pwm, vel, 1, full=True)
# print(lines)

# Using scipy library
# slope, intercept, r_value, p_value, std_err = stats.linregress(pwm, vel)
# print(slope, intercept, r_value, p_value, std_err)

m, b = np.polyfit(pwm, vel, 1)
fitting_string = f"y = {m:.4f} x + {b:.4f}"
plt.plot(pwm, m * pwm + b, label=fitting_string)
plt.plot(pwm, vel, 'o')
plt.legend()
plt.show()