import matplotlib.pyplot as plt
import numpy as np
from load_data import load_data

# x = np.arange(0.0, 2.0, 0.01)
# y = 1 + np.sin(2*np.pi*x)
# z = 1 + np.cos(2*np.pi*x)


data_nl = load_data("Cartpole_LQR.dat", delim=",")
data_nlg = load_data("Cartpole_LQG.dat", delim=",")


    




fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2,2)

ax1.plot(data_nl[:,0],data_nl[:,1])
ax2.plot(data_nl[:,0],data_nl[:,2])
ax3.plot(data_nl[:,0],data_nl[:,3])
ax4.plot(data_nl[:,0],data_nl[:,4])

fig2, ((ax5, ax6), (ax7, ax8)) = plt.subplots(2,2)

ax5.plot(data_nlg[:,0],data_nlg[:,1])
ax6.plot(data_nlg[:,0],data_nlg[:,2])
ax7.plot(data_nlg[:,0],data_nlg[:,3])
ax8.plot(data_nlg[:,0],data_nlg[:,4])
ax5.plot(data_nlg[:,0],data_nlg[:,5],'--')
ax6.plot(data_nlg[:,0],data_nlg[:,6],'--')
ax7.plot(data_nlg[:,0],data_nlg[:,7],'--')
ax8.plot(data_nlg[:,0],data_nlg[:,8],'--')



ax1.set_ylabel("x (m)")
ax2.set_ylabel("xdot (m/s)")
ax3.set_ylabel("th (rad)")
ax4.set_ylabel("thdot (rad/s)")
ax5.set_ylabel("x (m)")
ax6.set_ylabel("xdot (m/s)")
ax7.set_ylabel("th (rad)")
ax8.set_ylabel("thdot (rad/s)")


# extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
# sz = extents[:,1] - extents[:,0]
# centers = np.mean(extents, axis=1)
# maxsize = max(abs(sz))
# r = maxsize/2
# for ctr, dim in zip(centers, 'xyz'):
    # getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)


plt.show()