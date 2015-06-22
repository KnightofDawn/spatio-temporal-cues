#!/usr/bin/env python

import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
import numpy as np
from scipy.stats import vonmises

# x = np.linspace(0, 5, 111)
# y = multivariate_normal.pdf(x, mean=2.5, cov=0.5)

# fig1 = plt.figure()
# ax = fig1.add_subplot(111)
# ax.plot(x, y)
# plt.show()


# x = np.linspace(0, 5, 10, endpoint=False)
# y = multivariate_normal.pdf(x, mean=2.5, cov=0.5); y
# array([ 0.00108914,  0.01033349,  0.05946514,  0.20755375,  0.43939129,
#         0.56418958,  0.43939129,  0.20755375,  0.05946514,  0.01033349])
# fig1 = plt.figure()
# ax = fig1.add_subplot(111)
# ax.plot(x, y)
# The input quantiles can be any shape of array, as long as the last axis labels the components. This allows us for instance to display the frozen pdf for a non-isotropic random variable in 2D as follows:

kappa = 3.99390425811
rv = vonmises([1,1],loc=[0,0])
print rv.pdf([0,0])


# x, y = np.mgrid[-1:1:.01, -1:1:.01]
# pos = np.empty(x.shape + (2,))
# pos[:, :, 0] = x; pos[:, :, 1] = y
# # rv = multivariate_normal([0.5, 0.5], [1.0, 1.0], allow_singular=True)



# fig2 = plt.figure()
# ax2 = fig2.add_subplot(111)
# ax2.contourf(x, y, rv.pdf(pos))
plt.show()

