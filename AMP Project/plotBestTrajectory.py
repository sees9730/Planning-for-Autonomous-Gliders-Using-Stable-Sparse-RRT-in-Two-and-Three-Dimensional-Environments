import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from PIL import Image
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle

def add_airplane_image(ax, x, z, angle, zoom=0.001):
    """
    Create a rotated airplane image at (x, z) with the specified angle.
    
    Parameters:
        ax (matplotlib.axes.Axes): The axes to add the image to.
        x (float): The x-coordinate where the image will be placed.
        z (float): The z-coordinate where the image will be placed.
        angle (float): The rotation angle in degrees.
        zoom (float): The zoom factor for the image size.
    
    Returns:
        AnnotationBbox: The annotation box containing the airplane image.
    """
    # Load the airplane image using PIL
    airplane_img = Image.open('airplane.png').convert("RGBA")
    
    # Rotate the image around its center
    rotated_img = airplane_img.rotate(angle, expand=True)  # Negative to match matplotlib's rotation direction
    
    # Convert the rotated image to a format compatible with OffsetImage
    rotated_img_np = np.array(rotated_img)
    
    # Create an OffsetImage with the rotated image
    image = OffsetImage(rotated_img_np, zoom=zoom)
    
    # Create an AnnotationBbox at the specified (x, z) location
    ab = AnnotationBbox(image, (x, z), frameon=False, pad=0.0)
    
    return ab

# Set save to True if you want to save the plots
save = False
title = 'SSTstar/More_Control'
title_title ='SST*'

# Load the CSV files
file_path = 'best_path_sststar.csv'
data = pd.read_csv(file_path)

obstacles_file_path = 'obstacles_sststar.csv'
obstacles_data = pd.read_csv(obstacles_file_path)

# Extract x and z columns
x = data['x']
y = data['y']
z = -data['z']  # Negate z for plotting

# Convert theta from radians to degrees
theta = np.degrees(data['theta'])  # Ensure 'theta' is in radians initially

# Optional: Subsample data to reduce the number of frames (adjust 'n' as needed)
n = 1  # Change to 1 for no subsampling
x = x[::n].reset_index(drop=True)
y = y[::n].reset_index(drop=True)
z = z[::n].reset_index(drop=True)
theta = theta[::n].reset_index(drop=True)

# Fit a line (degree=1)
coefficients = np.polyfit(x, z, 1)
line = np.poly1d(coefficients)

# Predicted values
z_pred = line(x)

# Calculate residuals
residuals = z - z_pred

# Measure straightness (e.g., standard deviation of residuals)
straightness = np.std(residuals)

if straightness < 1e-5:
    straightness = 0

# Prepare the figure and axes
fig, ax = plt.subplots(2, 1,figsize=(10, 8))

# Plot the full trajectory in light gray for reference
ax[0].plot(x, y, z, linestyle='-', label='Trajectory')
# ax.plot([], [], linestyle='-', label=f'Straightness: {straightness:.2f}')

# Plot each obstacle as a red polygon, excluding the wind circle and wind vectors
unique_obstacles = obstacles_data['ObstacleID'].unique()
WIND_OBSTACLE_ID = 999  # The unique ID for wind circle
WIND_VECTOR_BASE_ID = 1000    # Starting ObstacleID for wind vectors

for obstacle_id in unique_obstacles:
    if obstacle_id == WIND_OBSTACLE_ID or obstacle_id >= WIND_VECTOR_BASE_ID:
        continue  # Skip wind data for now
    obstacle_vertices = obstacles_data[obstacles_data['ObstacleID'] == obstacle_id]
    obstacle_x = obstacle_vertices['x'].values
    obstacle_z = -obstacle_vertices['z'].values
    center_x = np.mean(obstacle_x)
    center_z = np.mean(obstacle_z)
    angles = np.arctan2(obstacle_z - center_z, obstacle_x - center_x)
    sorted_indices = np.argsort(angles)
    obstacle_x = obstacle_x[sorted_indices]
    obstacle_z = obstacle_z[sorted_indices]
    obstacle_x = np.append(obstacle_x, obstacle_x[0])
    obstacle_z = np.append(obstacle_z, obstacle_z[0])
    ax[0].fill(obstacle_x, obstacle_z, color='red', alpha=0.5, label='Obstacle' if obstacle_id == unique_obstacles[0] else "")

# # Plot Wind Circle
# wind_obstacle = obstacles_data[obstacles_data['ObstacleID'] == WIND_OBSTACLE_ID]
# if not wind_obstacle.empty:
#     wind_x = wind_obstacle['x'].values
#     wind_z = -wind_obstacle['z'].values  # Negate z for plotting

#     # Calculate center and radius
#     center_x = np.mean(wind_x)
#     center_z = np.mean(wind_z)
#     # Calculate average radius
#     distances = np.sqrt((wind_x - center_x)**2 + (wind_z - center_z)**2)
#     radius = np.mean(distances)

#     # Create a Circle patch
#     wind_circle = Circle((center_x, center_z), radius, color='red', fill=True, alpha = 0.2, linestyle='--', label='Wind Region')
#     ax[0].add_patch(wind_circle)

#     # Plot Multiple Wind Vectors
#     wind_vectors = obstacles_data[obstacles_data['ObstacleID'] >= WIND_VECTOR_BASE_ID]
#     if not wind_vectors.empty:
#         # Sort wind_vectors by ObstacleID to ensure pairing
#         wind_vectors_sorted = wind_vectors.sort_values(by=['ObstacleID', 'VertexID'])
#         grouped = wind_vectors_sorted.groupby('ObstacleID')

#         for obstacle_id, group in grouped:
#             if len(group) < 2:
#                 continue  # Need at least two points to plot a vector
#             # Extract start and end points
#             wind_start = group[group['VertexID'] == 0].iloc[0]
#             wind_end = group[group['VertexID'] == 1].iloc[0]
#             wind_start_x, wind_start_z = wind_start['x'], -wind_start['z']
#             wind_end_x, wind_end_z = wind_end['x'], -wind_end['z']

#             # Calculate wind vector components
#             wind_dx = wind_end_x - wind_start_x
#             wind_dz = wind_end_z - wind_start_z

#             # Plot the wind vector as a magenta arrow
#             ax[0].arrow(wind_start_x, wind_start_z, wind_dx, wind_dz,
#                      head_width=2, head_length=1, fc='red', ec='red',
#                      length_includes_head=True, label='Wind Vector' if obstacle_id == WIND_VECTOR_BASE_ID else "")
#             # Note: Only label once to avoid duplicate legend entries



factor = 1
# plt.figure(figsize=(10, 6))
ax[1].scatter(np.arange(0, len(data['delta_e']), 1), data['delta_e'], linestyle='-')
ax[1].axhline(y=-factor*(np.pi/9) , color='red', linestyle='--')
ax[1].axhline(y=factor*(np.pi/9), color='red', linestyle='--')
ax[1].set_ylim([-factor*(np.pi/9) - 0.1, factor*(np.pi/9) + 0.1])
ax[1].legend()
ax[1].set_ylabel('Elevator Deflection (rad)')
ax[1].set_xlabel('Time (s)')
ax[1].set_title('Elevator Deflection to Reach Target')
# plt.ylim([-np.pi, np.pi])
ax[1].grid(True)
# plt.savefig(f'FinalPlots/{title}ElevatorDeflection.png', dpi = 300)

# Set plot titles and labels
ax[0].set_title(f'{title_title} Trajectory')
ax[0].set_xlabel('x (Horizontal Distance)')
ax[0].set_ylabel('z (Vertical Distance)')
ax[0].grid(True)

# Initialize the airplane image at the first position
for i in range(0, len(x), 50    ):
    initial_ab = add_airplane_image(ax, x.iloc[i], z.iloc[i], theta.iloc[i], zoom=0.15)
    ax[0].add_artist(initial_ab)

# If you want to show the trajectory building up over time, initialize a Line2D object
# traj_line, = ax.plot([], [], linewidth=2, label='Trajectory')

# Handle legends to avoid duplicate labels
handles, labels = ax[0].get_legend_handles_labels()
unique_labels = {}
for h, l in zip(handles, labels):
    if l not in unique_labels:
        unique_labels[l] = h
ax[0].legend(unique_labels.values(), unique_labels.keys())

# Define a list to hold the current AnnotationBbox
current_ab = [initial_ab]

def update(frame):
    """
    Update function for animation.
    
    Parameters:
        frame (int): The current frame index.
    
    Returns:
        list: A list of artists to be redrawn.
    """
    # Remove the previous airplane image
    current_ab[0].remove()
    
    # Add the new airplane image
    ab = add_airplane_image(ax, x.iloc[frame], z.iloc[frame], theta.iloc[frame], zoom=0.15)
    ax.add_artist(ab)
    current_ab[0] = ab  # Update the reference
    
    # Update the trajectory line to include the current position
    traj_line.set_data(x.iloc[:frame+1], z.iloc[:frame+1])
    
    return [ab, traj_line]

# Determine the number of frames
num_frames = len(x)

# Create the animation
# anim = FuncAnimation(fig, update, frames=num_frames, interval=50, blit=True)

# Save the animation as a GIF
# anim.save('airplane_trajectory.gif', writer=PillowWriter(fps=5))

# Optionally, display the plot
# if False:


# # Get the indices at which the plane passes through the wind circle
# wind_x = wind_obstacle['x'].values
# wind_z = -wind_obstacle['z'].values  # Negate z for plotting

# # Calculate center and radius
# center_x = np.mean(wind_x)
# center_z = -np.mean(wind_z)

# wind_circle_indices = np.where(((data['x'] - center_x)**2 + (data['z'] - center_z)**2) <= radius**2)

# first_index = wind_circle_indices[0][0]
# last_index = wind_circle_indices[0][-1]



# ax[2].plot(data['theta'], marker='.', linestyle='-')
# ax[2].axvspan(first_index, last_index, alpha=0.2, color='red')
# ax[2].grid(True)
# ax[2].set_ylabel('Theta (rad)')
# ax[2].set_xlabel('Time (s)')


plt.tight_layout()
plt.savefig(f'FinalPlots/{title}Trajectory.png', dpi = 300)
plt.show()

# The rest of your plotting code (Euler angles, velocities, rates) can remain unchanged
# plt.plot(x, y)
# plt.show()


# # Get the indices at which the plane passes through the wind circle
# wind_x = wind_obstacle['x'].values
# wind_z = -wind_obstacle['z'].values  # Negate z for plotting

# # Calculate center and radius
# center_x = np.mean(wind_x)
# center_z = -np.mean(wind_z)

# wind_circle_indices = np.where(((data['x'] - center_x)**2 + (data['z'] - center_z)**2) <= radius**2)

# first_index = wind_circle_indices[0][0]
# last_index = wind_circle_indices[0][-1]

# Plot phi, theta, and psi
fig, ax = plt.subplots(3, figsize=(10, 6))

ax[0].plot(data['phi'], marker='.', linestyle='-')
# ax[0].axvspan(first_index, last_index, alpha=0.2, color='red')
ax[1].plot(data['theta'], marker='.', linestyle='-')
# ax[1].axvspan(first_index, last_index, alpha=0.2, color='red')
ax[2].plot(data['psi'], marker='.', linestyle='-')
# ax[2].axvspan(first_index, last_index, alpha=0.2, color='red')

# Add labels and title
ax[0].set_ylabel('phi (rad)')
ax[1].set_ylabel('theta (rad)')
ax[2].set_ylabel('psi (rad)')

ax[0].grid(True)
ax[1].grid(True)
ax[2].grid(True)

ax[2].set_xlabel('Time')
plt.suptitle('Euler Angles With No Control Input')
if save:
    plt.savefig(f'FinalPlots/{title}Euler.png')
plt.show()


# Plot u, v, and w
fig, ax = plt.subplots(3, figsize=(10, 6))

ax[0].plot(data['u'], marker='.', linestyle='-')
# ax[0].axvspan(first_index, last_index, alpha=0.2, color='red')
ax[1].plot(data['v'], marker='.', linestyle='-')
# ax[1].axvspan(first_index, last_index, alpha=0.2, color='red')
ax[2].plot(data['w'], marker='.', linestyle='-')
# ax[2].axvspan(first_index, last_index, alpha=0.2, color='red')

# Add labels and title
ax[0].set_ylabel('u (m/s)')
ax[1].set_ylabel('v (m/s)')
ax[2].set_ylabel('w (m/s)')

ax[0].grid(True)
ax[1].grid(True)
ax[2].grid(True)

ax[2].set_xlabel('Time')
plt.suptitle('Body Velocity With No Control Input')
if save:
    plt.savefig(f'FinalPlots/{title}BodyVelocity.png')
plt.show()

# Plot p, q, and r
fig, ax = plt.subplots(3, figsize=(10, 6))

ax[0].plot(data['p'], marker='.', linestyle='-')
# ax[0].axvspan(first_index, last_index, alpha=0.2, color='red')
ax[1].plot(data['q'], marker='.', linestyle='-')
# ax[1].axvspan(first_index, last_index, alpha=0.2, color='red')
ax[2].plot(data['r'], marker='.', linestyle='-')
# ax[2].axvspan(first_index, last_index, alpha=0.2, color='red')

# Add labels and title
ax[0].set_ylabel('p (rad/s)')
ax[1].set_ylabel('q (rad/s)')
ax[2].set_ylabel('r (rad/s)')

ax[0].grid(True)
ax[1].grid(True)
ax[2].grid(True)

ax[2].set_xlabel('Time')
plt.suptitle('Body Rate With No Control Input')
if save:
    plt.savefig(f'FinalPlots/{title}BodyRate.png')
plt.show()



# # Plot delta_e, delta_a, and delta_r
# fig, ax = plt.subplots(3, figsize=(10, 6))

# ax[0].plot(data['delta_e'], marker='.', linestyle='-')
# ax[0].axvspan(first_index, last_index, alpha=0.2, color='red')
# ax[1].plot(data['delta_a'], marker='.', linestyle='-')
# ax[1].axvspan(first_index, last_index, alpha=0.2, color='red')
# ax[2].plot(data['delta_r'], marker='.', linestyle='-')
# ax[2].axvspan(first_index, last_index, alpha=0.2, color='red')

# # Add labels and title
# ax[0].set_ylabel('Elevator (rad)')
# ax[1].set_ylabel('Aileron (rad)')
# ax[2].set_ylabel('Rudder (rad)')

# ax[0].grid(True)
# ax[1].grid(True)
# ax[2].grid(True)

# ax[2].set_xlabel('Time')
# plt.suptitle('Body Rate With No Control Input')
# if save:
#     plt.savefig(f'FinalPlots/{title}BodyRate.png')
# plt.show()
