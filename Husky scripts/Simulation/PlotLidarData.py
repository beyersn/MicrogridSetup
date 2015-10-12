# python experimental tests for Husky

import numpy as np
import glob
import matplotlib.pyplot as plt
from math import pi


for file in glob.glob("ScanData*.npy"):
    data = np.load(file)
    print file,
    distances= data
    angles = np.linspace(-pi/4,5*pi/4 ,len(distances))
    x = []
    y = []
    for i in range(0, len(distances)):
        x.append(distances[i]*np.cos(angles[i]))
        y.append(distances[i]*np.sin(angles[i]))

    f0 = plt.figure()

    ax0 = f0.add_subplot(111)
    ax0.plot(x,y, 'ro')
    ax0.axis('equal')
    plt.draw()
    plt.pause(.1)
    raw_input("<Hit Enter To Close>")
    plt.close(f0)
