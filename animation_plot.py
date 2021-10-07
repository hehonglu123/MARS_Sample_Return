
import matplotlib.pyplot as plt
import numpy as np
  
x = np.linspace(0, 10*np.pi, 100)
y = np.sin(x)
  
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot(x, y, 'b-')
line2, =ax.plot(x,y+0.2, 'r-')
  
for phase in np.linspace(0, 10*np.pi, 100):
    line1.set_ydata(np.sin(0.5 * x + phase))
    line2.set_ydata(np.sin(0.5 * x + phase)+0.2)
    fig.canvas.draw()
    fig.canvas.flush_events()