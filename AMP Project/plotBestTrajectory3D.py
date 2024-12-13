import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D  # Import the 3D toolkit
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def add_circle(ax, x_center, y_center, radius, z_plane=0, color='green', alpha=0.5):
    """
    Add a circle in the x-y plane to the 3D plot.
    
    Parameters:
        ax (mpl_toolkits.mplot3d.Axes3D): The 3D axes to add the circle to.
        x_center (float): x-coordinate of the circle's center.
        y_center (float): y-coordinate of the circle's center.
        radius (float): Radius of the circle.
        z_plane (float): z-coordinate where the circle lies (default is 0).
        color (str): Color of the circle.
        alpha (float): Transparency level of the circle.
    """
    # Generate points for the circle
    theta = np.linspace(0, 2 * np.pi, 100)
    x_circle = x_center + radius * np.cos(theta)
    y_circle = y_center + radius * np.sin(theta)
    z_circle = np.full_like(x_circle, z_plane)  # Circle lies in the z-plane
    
    # Add the circle to the plot
    verts = [list(zip(x_circle, y_circle, z_circle))]
    poly = Poly3DCollection(verts, color=color, alpha=alpha)
    
    # Add the filled circle to the plot
    ax.add_collection3d(poly)


# Function to add a 3D airplane marker
def add_airplane_marker(ax, x, y, z, angle, size=100):
    """
    Add a 3D scatter marker to represent the airplane.
    
    Parameters:
        ax (mpl_toolkits.mplot3d.Axes3D): The 3D axes to add the marker to.
        x (float): The x-coordinate.
        y (float): The y-coordinate.
        z (float): The z-coordinate.
        angle (float): The rotation angle in degrees (not visually represented in marker).
        size (int): Marker size.
    """
    # Use a triangular marker to indicate direction
    ax.scatter(x, y, z, marker=(3, 0, angle), color='blue', s=size)

# Set save to True if you want to save the plots
save = False
title = 'SSTstar/10CI10'

# Load the CSV files
file_path = 'best_path_sststar.csv'
data = pd.read_csv(file_path)

obstacles_file_path = 'obstacles_sststar.csv'
obstacles_data = pd.read_csv(obstacles_file_path)

# Extract x, y, and z columns
x = data['x']
# Check if 'y' exists in the data; if not, set y to zero
if 'y' in data.columns:
    y = data['y']
else:
    # If 'y' is not present, you can derive it or set it to zero
    # Example: Assuming no lateral movement
    y = np.zeros_like(x)
z = -data['z']  # Negate z for plotting

# Convert theta from radians to degrees
theta = np.degrees(data['theta'])  # Ensure 'theta' is in radians initially

# Optional: Subsample data to reduce the number of frames (adjust 'n' as needed)
n = 1  # Change to 1 for no subsampling
x = x[::n].reset_index(drop=True)
y = y[::n].reset_index(drop=True)
z = z[::n].reset_index(drop=True)
theta = theta[::n].reset_index(drop=True)

# Prepare the figure and 3D axes
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot the full trajectory in light gray for reference
ax.plot(x, y, z, marker='.', linestyle='-')

circles = [
    {"x_center": 200, "y_center": -50, "radius": 10, "z_plane": 0},
    # {"x_center": 20, "y_center": 25, "radius": 3, "z_plane": 0},
]

# Add circles to the plot
for circle in circles:
    add_circle(ax, circle["x_center"], circle["y_center"], circle["radius"], z_plane=circle["z_plane"])


# Plot each obstacle as a red polygon, excluding the wind circle and wind vectors
unique_obstacles = obstacles_data['ObstacleID'].unique()
WIND_OBSTACLE_ID = 999  # The unique ID for wind circle
WIND_VECTOR_BASE_ID = 1000    # Starting ObstacleID for wind vectors

for obstacle_id in unique_obstacles:
    if obstacle_id == WIND_OBSTACLE_ID or obstacle_id >= WIND_VECTOR_BASE_ID:
        continue  # Skip wind data for now
    obstacle_vertices = obstacles_data[obstacles_data['ObstacleID'] == obstacle_id]
    obstacle_x = obstacle_vertices['x'].values
    # Handle 'y' for obstacles; set to zero if not present
    if 'y' in obstacle_vertices.columns:
        obstacle_y = obstacle_vertices['y'].values
    else:
        obstacle_y = np.zeros_like(obstacle_x)
    obstacle_z = -obstacle_vertices['z'].values
    center_x = np.mean(obstacle_x)
    center_y = np.mean(obstacle_y)
    center_z = np.mean(obstacle_z)
    angles = np.arctan2(obstacle_z - center_z, obstacle_x - center_x)
    sorted_indices = np.argsort(angles)
    obstacle_x = obstacle_x[sorted_indices]
    obstacle_y = obstacle_y[sorted_indices]
    obstacle_z = obstacle_z[sorted_indices]
    # Close the polygon by appending the first vertex at the end
    obstacle_x = np.append(obstacle_x, obstacle_x[0])
    obstacle_y = np.append(obstacle_y, obstacle_y[0])
    obstacle_z = np.append(obstacle_z, obstacle_z[0])
    # Create a list of vertices for Poly3DCollection
    verts = [list(zip(obstacle_x, obstacle_y, obstacle_z))]
    poly = Poly3DCollection(verts, facecolors='red', alpha=0.5, label='Obstacle' if obstacle_id == unique_obstacles[0] else "")
    ax.add_collection3d(poly)

# # Plot Wind Circle
# wind_obstacle = obstacles_data[obstacles_data['ObstacleID'] == WIND_OBSTACLE_ID]
# if not wind_obstacle.empty:
#     wind_x = wind_obstacle['x'].values
#     # Handle 'y' for wind; set to zero if not present
#     if 'y' in wind_obstacle.columns:
#         wind_y = wind_obstacle['y'].values
#     else:
#         wind_y = np.zeros_like(wind_x)
#     wind_z = -wind_obstacle['z'].values  # Negate z for plotting

#     # Calculate center and radius
#     center_x = np.mean(wind_x)
#     center_y = np.mean(wind_y)
#     center_z = np.mean(wind_z)
#     # Calculate average radius
#     distances = np.sqrt((wind_x - center_x)**2 + (wind_y - center_y)**2 + (wind_z - center_z)**2)
#     radius = np.mean(distances)

#     # Create a sphere for wind region
#     u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
#     sphere_x = center_x + radius * np.cos(u) * np.sin(v)
#     sphere_y = center_y + radius * np.sin(u) * np.sin(v)
#     sphere_z = center_z + radius * np.cos(v)
#     ax.plot_wireframe(sphere_x, sphere_y, sphere_z, color='red', alpha=0.2, label='Wind Region')

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
#             wind_start_x, wind_start_y = wind_start['x'], wind_start.get('y', 0)
#             wind_start_z = -wind_start['z']
#             wind_end_x, wind_end_y = wind_end['x'], wind_end.get('y', 0)
#             wind_end_z = -wind_end['z']

#             # Calculate wind vector components
#             wind_dx = wind_end_x - wind_start_x
#             wind_dy = wind_end_y - wind_start_y
#             wind_dz = wind_end_z - wind_start_z

#             # Plot the wind vector as a red arrow
#             ax.quiver(wind_start_x, wind_start_y, wind_start_z,
#                       wind_dx, wind_dy, wind_dz,
#                       length=1, normalize=True, color='red',
#                       arrow_length_ratio=0.2, label='Wind Vector' if obstacle_id == WIND_VECTOR_BASE_ID else "")
#             # Note: Only label once to avoid duplicate legend entries

# Set plot titles and labels
# ax.axis(xlim = (x.min(), x.max()), ylim = (y.min(), y.max()), zlim = (z.min(), z.max()))
ax.set_xlim([x.min(), 200 + 10])
ax.set_ylim([-55, 0])
ax.set_title('3D Trajectory of Glider with 3D Control Space')
ax.set_xlabel('x (Horizontal Distance)')
ax.set_ylabel('y (Lateral Distance)')
ax.set_zlabel('z (Vertical Distance)')
ax.grid(True)

# Handle legends to avoid duplicate labels
handles, labels = ax.get_legend_handles_labels()
unique_labels = {}
for h, l in zip(handles, labels):
    if l not in unique_labels and l != '_nolegend_':
        unique_labels[l] = h
ax.legend(unique_labels.values(), unique_labels.keys())

# Initialize the airplane marker at the first position
initial_x = x.iloc[0]
initial_y = y.iloc[0]
initial_z = z.iloc[0]
initial_angle = theta.iloc[0]
# Plot the initial airplane marker
# airplane_marker, = ax.plot([initial_x], [initial_y], [initial_z],
#                            marker=(3, 0, initial_angle), markersize=15,
#                            label='Airplane')

# Initialize the trajectory line
# traj_line, = ax.plot([], [], [], color='blue', linewidth=2, label='Trajectory')

# Define the update function for animation
def update(frame):
    """
    Update function for animation.
    
    Parameters:
        frame (int): The current frame index.
    
    Returns:
        list: A list of artists to be redrawn.
    """
    # Update the airplane marker position
    current_x = x.iloc[frame]
    current_y = y.iloc[frame]
    current_z = z.iloc[frame]
    current_angle = theta.iloc[frame]
    
    # Update the airplane marker data
    airplane_marker.set_data([current_x], [current_y])
    airplane_marker.set_3d_properties([current_z])
    airplane_marker.set_marker((3, 0, current_angle))  # Update angle if needed
    
    # Update the trajectory line to include the current position
    traj_line.set_data(x.iloc[:frame+1], y.iloc[:frame+1])
    traj_line.set_3d_properties(z.iloc[:frame+1])
    
    return [airplane_marker, traj_line]

# Determine the number of frames
num_frames = len(x)

# Create the animation
anim = FuncAnimation(fig, update, frames=num_frames, interval=50, blit=False)

# Save the animation as a GIF
anim.save('airplane_trajectory_3d.gif', writer=PillowWriter(fps=5))

# Optionally, save the final plot
# if save:
# plt.savefig(f'FinalPlots/{title}_Trajectory_3D.png', dpi = 200)
plt.show()

# The rest of your plotting code (Euler angles, velocities, rates) can remain unchanged

# # Get the indices at which the plane passes through the wind circle
# wind_x = wind_obstacle['x'].values
# # Handle 'y' for wind; set to zero if not present
# if 'y' in wind_obstacle.columns:
#     wind_y = wind_obstacle['y'].values
# else:
#     wind_y = np.zeros_like(wind_x)
# wind_z = -wind_obstacle['z'].values  # Negate z for plotting

# # Calculate center and radius
# center_x = np.mean(wind_x)
# center_y = np.mean(wind_y)
# center_z = np.mean(wind_z)

# wind_circle_indices = np.where(((data['x'] - center_x)**2 + (data['y'] - center_y)**2 + (data['z'] - center_z)**2) <= radius**2)

# first_index = wind_circle_indices[0][0]
# last_index = wind_circle_indices[0][-1]














plt.plot(data['x'], data['y'])
plt.title('X vs. Y')
plt.grid()
plt.show()
plt.plot(data['x'], data['z'])
plt.title('X vs. Z')
plt.grid()
plt.show()





# # Plot phi, theta, and psi
# fig_angles, ax_angles = plt.subplots(3, figsize=(10, 8))

# ax_angles[0].plot(data['phi'], marker='.', linestyle='-')
# # ax_angles[0].axvspan(first_index, last_index, alpha=0.2, color='red')
# ax_angles[1].plot(data['theta'], marker='.', linestyle='-')
# # ax_angles[1].axvspan(first_index, last_index, alpha=0.2, color='red')
# ax_angles[2].plot(data['psi'], marker='.', linestyle='-')
# # ax_angles[2].axvspan(first_index, last_index, alpha=0.2, color='red')

# # Add labels and title
# ax_angles[0].set_ylabel('phi (rad)')
# ax_angles[1].set_ylabel('theta (rad)')
# ax_angles[2].set_ylabel('psi (rad)')

# ax_angles[0].grid(True)
# ax_angles[1].grid(True)
# ax_angles[2].grid(True)

# ax_angles[2].set_xlabel('Time')
# plt.suptitle('Euler Angles With No Control Input')
# if save:
#     plt.savefig(f'FinalPlots/{title}_Euler.png')
# plt.show()

# # Plot u, v, and w
# fig_velocity, ax_velocity = plt.subplots(3, figsize=(10, 8))

# ax_velocity[0].plot(data['u'], marker='.', linestyle='-')
# # ax_velocity[0].axvspan(first_index, last_index, alpha=0.2, color='red')
# ax_velocity[1].plot(data['v'], marker='.', linestyle='-')
# # ax_velocity[1].axvspan(first_index, last_index, alpha=0.2, color='red')
# ax_velocity[2].plot(data['w'], marker='.', linestyle='-')
# # ax_velocity[2].axvspan(first_index, last_index, alpha=0.2, color='red')

# # Add labels and title
# ax_velocity[0].set_ylabel('u (m/s)')
# ax_velocity[1].set_ylabel('v (m/s)')
# ax_velocity[2].set_ylabel('w (m/s)')

# ax_velocity[0].grid(True)
# ax_velocity[1].grid(True)
# ax_velocity[2].grid(True)

# ax_velocity[2].set_xlabel('Time')
# plt.suptitle('Body Velocity With No Control Input')
# if save:
#     plt.savefig(f'FinalPlots/{title}_BodyVelocity.png')
# plt.show()

# # Plot p, q, and r
# fig_rates, ax_rates = plt.subplots(3, figsize=(10, 8))

# ax_rates[0].plot(data['p'], marker='.', linestyle='-')
# # ax_rates[0].axvspan(first_index, last_index, alpha=0.2, color='red')
# ax_rates[1].plot(data['q'], marker='.', linestyle='-')
# # ax_rates[1].axvspan(first_index, last_index, alpha=0.2, color='red')
# ax_rates[2].plot(data['r'], marker='.', linestyle='-')
# # ax_rates[2].axvspan(first_index, last_index, alpha=0.2, color='red')

# # Add labels and title
# ax_rates[0].set_ylabel('p (rad/s)')
# ax_rates[1].set_ylabel('q (rad/s)')
# ax_rates[2].set_ylabel('r (rad/s)')

# ax_rates[0].grid(True)
# ax_rates[1].grid(True)
# ax_rates[2].grid(True)

# ax_rates[2].set_xlabel('Time')
# plt.suptitle('Body Rate With No Control Input')
# if save:
#     plt.savefig(f'FinalPlots/{title}_BodyRate.png')
# plt.show()

# # Plot delta_e, delta_a, and delta_r
# fig_controls, ax_controls = plt.subplots(3, figsize=(10, 8))

# ax_controls[0].plot(data['delta_e'], marker='.', linestyle='-')
# # ax_controls[0].axvspan(first_index, last_index, alpha=0.2, color='red')
# ax_controls[1].plot(data['delta_a'], marker='.', linestyle='-')
# # ax_controls[1].axvspan(first_index, last_index, alpha=0.2, color='red')
# ax_controls[2].plot(data['delta_r'], marker='.', linestyle='-')
# # ax_controls[2].axvspan(first_index, last_index, alpha=0.2, color='red')

# # Add labels and title
# ax_controls[0].set_ylabel('Elevator (rad)')
# ax_controls[1].set_ylabel('Aileron (rad)')
# ax_controls[2].set_ylabel('Rudder (rad)')

# ax_controls[0].grid(True)
# ax_controls[1].grid(True)
# ax_controls[2].grid(True)

# ax_controls[2].set_xlabel('Time')
# plt.suptitle('Control Inputs With No Control Input')
# if save:
#     plt.savefig(f'FinalPlots/{title}_ControlInputs.png')
# plt.show()
