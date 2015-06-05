import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
import numpy as np

x = np.linspace(0, 5, 111)
y = multivariate_normal.pdf(x, mean=2.5, cov=0.5)

fig1 = plt.figure()
ax = fig1.add_subplot(111)
ax.plot(x, y)
plt.show()