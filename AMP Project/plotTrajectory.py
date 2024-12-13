# plot_states_individual_figures.py

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Necessary for 3D plotting
import os

def plot_state_groups(csv_file='trajectory.csv', save_plots=False, save_filenames=None):
    """
    Reads the trajectory CSV file and generates four separate 2D figures,
    each containing three related state parameter subplots, and one 3D trajectory plot.
    
    Parameters:
    - csv_file: str, path to the CSV file containing the trajectory data.
    - save_plots: bool, whether to save the plots as image files.
    - save_filenames: list of str, filenames for saving the plots if save_plots is True.
                      Should contain five filenames for the four 2D figures and one 3D plot.
    """
    
    # Read the CSV file into a pandas DataFrame
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: The file '{csv_file}' was not found.")
        return
    except pd.errors.EmptyDataError:
        print(f"Error: The file '{csv_file}' is empty.")
        return
    except pd.errors.ParserError:
        print(f"Error: The file '{csv_file}' does not appear to be in CSV format.")
        return
    
    # Check if all required columns are present
    required_columns = ['time','x','y','z','phi','theta','psi','u','v','w','p','q','r']
    if not all(column in df.columns for column in required_columns):
        print(f"Error: The CSV file must contain the following columns: {', '.join(required_columns)}")
        return
    
    # Define state groups for plotting
    state_groups = {
        'Position': ['x', 'y', 'z'],
        'Orientation': ['phi', 'theta', 'psi'],
        'Velocities': ['u', 'v', 'w'],
        'Angular_Rates': ['p', 'q', 'r']
    }
    
    # Set default save_filenames if not provided
    if save_filenames is None:
        save_filenames = [
            'Position_Figure.png',
            'Orientation_Figure.png',
            'Velocities_Figure.png',
            'Angular_Rates_Figure.png',
            '3D_Trajectory_Plot.png'
        ]
    
    # --- Generate Four Separate 2D Figures ---
    for idx, (group_name, variables) in enumerate(state_groups.items()):
        plt.figure(figsize=(15, 10))  # Adjust the figure size as needed
        
        for subplot_idx, var in enumerate(variables, start=1):
            ax = plt.subplot(3, 1, subplot_idx)  # 3 rows, 1 column, current subplot
            if subplot_idx == 3:
                ax.plot(df['time'], -df[var], label=var.capitalize(), color='k', linewidth=4)
            else:
                ax.plot(df['time'], df[var], label=var.capitalize(), color='k', linewidth=4)
            ax.set_ylabel(var.capitalize(), fontsize=20)
            ax.set_title(f'{group_name} - {var.capitalize()} Over Time')
            # ax.legend()
            ax.grid(True)
        
        plt.xlabel('Time (s)')
        plt.tight_layout()
        
        # # Save the plot if required
        # if save_plots and idx < len(save_filenames):
        #     plt.savefig(save_filenames[idx], dpi=300)
        #     print(f"Saved {group_name} figure as '{save_filenames[idx]}'")
        
        # Show the plot
        plt.show()
    
    
    # Use negative z values for the plot
    df['z_neg'] = -df['z']  # Add a column with negative z values for clarity

    plt.plot(df['x'], df['z_neg'], color = 'k')
    plt.ylabel('Z Position (meters)')
    plt.xlabel('X Position (meters)')
    plt.grid()
    plt.show()
    # --- Generate 3D Trajectory Plot with Negative Z ---
    plt.figure(figsize=(15, 10))
    ax3d = plt.axes(projection='3d')

    # Plot trajectory with negative z-axis
    ax3d.plot3D(df['x'], df['y'], df['z_neg'], label='Trajectory (Negative Z)', color='blue', linewidth=2)

    # Mark the starting point
    ax3d.scatter(df['x'].iloc[0], df['y'].iloc[0], df['z_neg'].iloc[0],
                color='green', marker='o', s=100, label='Start')

    # Mark the ending point
    ax3d.scatter(df['x'].iloc[-1], df['y'].iloc[-1], df['z_neg'].iloc[-1],
                color='red', marker='x', s=100, label='End')

    # Set labels
    ax3d.set_xlabel('X Position (meters)')
    ax3d.set_ylabel('Y Position (meters)')
    ax3d.set_zlabel('Negative Z Position (meters)')
    ax3d.set_title('3D Trajectory with Negative Z-Axis')
    ax3d.legend()

    # Optionally, set equal aspect ratio for all axes
    max_range = max(
        df['x'].max() - df['x'].min(),
        df['y'].max() - df['y'].min(),
        df['z_neg'].max() - df['z_neg'].min()
    ) / 2.0

    mid_x = (df['x'].max() + df['x'].min()) * 0.5
    mid_y = (df['y'].max() + df['y'].min()) * 0.5
    mid_z = (df['z_neg'].max() + df['z_neg'].min()) * 0.5
    ax3d.set_xlim(mid_x - max_range, mid_x + max_range)
    ax3d.set_ylim(mid_y - max_range, mid_y + max_range)
    ax3d.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.tight_layout()

    # # Save the 3D plot if required
    # if save_plots and len(save_filenames) >= 5:
    #     plt.savefig(save_filenames[4], dpi=300)
    #     print(f"Saved 3D Trajectory plot as '{save_filenames[4]}'")

    # Show the 3D plot
    plt.show()

if __name__ == "__main__":
    # Define filenames for saving plots if desired
    filenames = [
        'Position_Figure.png',
        'Orientation_Figure.png',
        'Velocities_Figure.png',
        'Angular_Rates_Figure.png',
        '3D_Trajectory_Plot.png'
    ]
    
    # Call the plotting function
    plot_state_groups(csv_file='trajectory.csv', save_plots=True, save_filenames=filenames)
