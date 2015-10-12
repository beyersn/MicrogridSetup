# python experimental tests for Husky

import numpy as np
import glob
import matplotlib.pyplot as plt


for file in glob.glob("*.npy"):
    data = np.load(file)[5:, :]
    print file,
    error_long = data[:, 0]
    error_lat = data[:, 1]
    ref_x = data[:, 2]
    ref_y = data[:, 3]
    pos_x = data[:, 4]
    pos_y = data[:, 5]
    print "pos y" + str(error_lat)
    pos_theta = data[:, 6]
    time = data[:, 7] - data[0, 7]

    # plt.plot(ref_x, ref_y, 'ro')
    # plt.gca().set_aspect('equal', adjustable='box')
    f0 = plt.figure()
    f1 = plt.figure()
    ax0 = f0.add_subplot(111)
    ax0.plot(ref_x, ref_y, 'ro', label='Reference')
    ax0.plot(pos_x[0], pos_y[0], 'bo')
    ax0.plot(pos_x, pos_y, 'b',label='Robot trajectory')
    ax0.legend()
    ax0.axis('equal')

    ax1 = f1.add_subplot(111)
    ax1.plot(time, error_long,label='Long error')
    ax1.plot(time, error_lat,label='Lat error')
    ax1.legend()
    plt.draw()
    plt.pause(.1)  # <-------
    raw_input("<Hit Enter To Close>")
    plt.close(f0)
    plt.close(f1)
