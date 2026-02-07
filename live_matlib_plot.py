import matplotlib.pyplot as plt
import numpy as np

# Global variables to maintain state across function calls
fig = None
ax = None
line = None

def update_plot(x, y):
    """
    Updates the live plot with a new data point (x, y).
    Initializes the plot on the first call and updates it on subsequent calls.
    
    Parameters:
    x (float or int): The new x-value.
    y (float or int): The new y-value.
    """
    global fig, ax, line
    
    if fig is None:
        # Initialize the plot on the first call
        plt.ion()  # Turn on interactive mode
        fig, ax = plt.subplots()
        line, = ax.plot([], [], 'b-', label='Trajectory')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Live Updating Trajectory')
        ax.legend()
        plt.show(block=False)  # Non-blocking show
    
    # Get current data
    current_x = line.get_xdata()
    current_y = line.get_ydata()
    
    # Append new point
    new_x = np.append(current_x, x)
    new_y = np.append(current_y, y)
    
    # Update the line data
    line.set_xdata(new_x)
    line.set_ydata(new_y)
    
    # Rescale the axes
    ax.relim()
    ax.autoscale_view()
    
    # Redraw the canvas
    fig.canvas.draw()
    fig.canvas.flush_events()