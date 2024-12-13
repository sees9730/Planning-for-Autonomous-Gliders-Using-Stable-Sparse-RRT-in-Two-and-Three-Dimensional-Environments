import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pandas as pd
import numpy as np

def plot_graph(csv_file):
    # Load data from CSV file
    graph_data = pd.read_csv(csv_file)

    # Extract relevant data
    node_ids = graph_data['node_id']
    x = graph_data['x']
    y = graph_data['y']
    z = -graph_data['z']
    neighbors = graph_data['neighbors']
    cost = graph_data['cost']

    # Determine whether to plot in 2D or 3D
    is_2d = all(graph_data['y'] == 0)

    # Normalize cost values for colormap
    norm = plt.Normalize(graph_data['cost'].min(), graph_data['cost'].max())
    colormap = cm.get_cmap('viridis')

    if is_2d:
        # 2D Plot
        fig, ax = plt.subplots(figsize=(12, 8))

        # Plot nodes with colors based on cost
        scatter = ax.scatter(x, z, c=cost, cmap=colormap, s=50, label='Nodes')

        # Plot connections
        for idx, neighbor_str in enumerate(neighbors):
            if pd.notna(neighbor_str):  # Ensure the neighbors field is not empty
                neighbor_ids = map(int, neighbor_str.split('|'))
                for neighbor_id in neighbor_ids:
                    neighbor_idx = graph_data[graph_data['node_id'] == neighbor_id].index[0]
                    ax.plot(
                        [x[idx], x[neighbor_idx]],
                        [z[idx], z[neighbor_idx]],
                        c='gray',
                        linestyle='--'
                    )

        # Add colorbar
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('Cost')

        # Labels and legend
        ax.grid(True)
        ax.set_title("2D Graph Structure Visualization", fontsize=16)
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Z-axis")

    else:
        # 3D Plot
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Plot nodes with colors based on cost
        scatter = ax.scatter(x, y, z, c=cost, cmap=colormap, s=50, label='Nodes')

        # Plot connections
        for idx, neighbor_str in enumerate(neighbors):
            if pd.notna(neighbor_str):  # Ensure the neighbors field is not empty
                neighbor_ids = map(int, neighbor_str.split('|'))
                for neighbor_id in neighbor_ids:
                    neighbor_idx = graph_data[graph_data['node_id'] == neighbor_id].index[0]
                    ax.plot(
                        [x[idx], x[neighbor_idx]],
                        [y[idx], y[neighbor_idx]],
                        [z[idx], z[neighbor_idx]],
                        c='gray',
                        linestyle='--'
                    )

        # Add colorbar
        cbar = plt.colorbar(scatter, ax=ax, pad=0.1)
        cbar.set_label('Cost')

        # Labels and legend
        ax.set_title("3D Graph Structure Visualization", fontsize=16)
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("Z-axis")

    # Show plot
    plt.show()

def plot_witness_set_with_radius(csv_file, radius):
    """
    Plot the witness set with coverage radius for each witness.

    Args:
        csv_file (str): Path to the CSV file containing the witness set.
        radius (float): Coverage radius to visualize.
    """
    # Load data from CSV file
    witness_data = pd.read_csv(csv_file)

    # Extract witness data
    witness_x = witness_data['witness_x']
    witness_y = witness_data['witness_y']
    witness_z = witness_data['witness_z']
    witness_cost = witness_data['witness_cost']

    # Extract representative data
    rep_x = witness_data['rep_x']
    rep_y = witness_data['rep_y']
    rep_z = witness_data['rep_z']
    rep_cost = witness_data['rep_cost']

    # Determine if data is 2D or 3D
    is_2d = np.all(witness_y == 0) and np.all(rep_y == 0)

    if is_2d:
        # 2D Plot
        fig, ax = plt.subplots(figsize=(12, 8))

        # Plot witnesses
        scatter = ax.scatter(witness_x, -witness_z, c=witness_cost, cmap='viridis', s=50, label='Witnesses')

        # Plot connections to representatives
        for i in range(len(witness_x)):
            if not pd.isna(rep_x[i]):
                ax.plot(
                    [witness_x[i], rep_x[i]],
                    [witness_z[i], rep_z[i]],
                    c='gray', linestyle='--'
                )

        # Add circles for coverage radius
        for i in range(len(witness_x)):
            circle = plt.Circle((witness_x[i],-witness_z[i]), radius, color='blue', fill=False, linestyle='--', alpha=0.5)
            ax.add_artist(circle)

        # Add colorbar
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('Witness Cost')

        # Labels and legend
        ax.grid(True)
        ax.set_title("2D Witness Set Visualization with Coverage Radius", fontsize=16)
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Z-axis")
        ax.legend()

    else:
        # 3D Plot
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Plot witnesses
        scatter = ax.scatter(witness_x, witness_y, witness_z, c=witness_cost, cmap='viridis', s=50, label='Witnesses')

        # Plot connections to representatives
        for i in range(len(witness_x)):
            if not pd.isna(rep_x[i]):
                ax.plot(
                    [witness_x[i], rep_x[i]],
                    [witness_y[i], rep_y[i]],
                    [witness_z[i], rep_z[i]],
                    c='gray', linestyle='--'
                )

        # Add spheres for coverage radius
        u = np.linspace(0, 2 * np.pi, 50)
        v = np.linspace(0, np.pi, 50)
        for i in range(len(witness_x)):
            x_sphere = radius * np.outer(np.cos(u), np.sin(v)) + witness_x[i]
            y_sphere = radius * np.outer(np.sin(u), np.sin(v)) + witness_y[i]
            z_sphere = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + witness_z[i]
            ax.plot_wireframe(x_sphere, y_sphere, z_sphere, color='blue', alpha=0.2)

        # Add colorbar
        cbar = plt.colorbar(scatter, ax=ax, pad=0.1)
        cbar.set_label('Witness Cost')

        # Labels and legend
        ax.set_title("3D Witness Set Visualization with Coverage Radius", fontsize=16)
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("Z-axis")
        ax.legend()

    # Show plot
    plt.show()

def plot_witness_set_with_nodes(csv_file, node_csv_file, radius):
    """
    Plot the witness set, representatives, and nodes with coverage radius.

    Args:
        csv_file (str): Path to the CSV file containing the witness set.
        node_csv_file (str): Path to the CSV file containing the nodes in the graph.
        radius (float): Coverage radius to visualize.
    """
    # Load data from CSV files
    witness_data = pd.read_csv(csv_file)
    node_data = pd.read_csv(node_csv_file)

    # Extract witness data
    witness_x = witness_data['witness_x']
    witness_y = witness_data['witness_y']
    witness_z = -witness_data['witness_z']  # Use -z

    # Extract representative data
    rep_x = witness_data['rep_x']
    rep_y = witness_data['rep_y']
    rep_z = -witness_data['rep_z']  # Use -z

    # Extract node data
    node_x = node_data['x']
    node_y = node_data['y']
    node_z = -node_data['z']  # Use -z
    node_cost = node_data['cost']  # Node costs for colorbar

    # Determine if data is 2D or 3D
    is_2d = np.all(witness_y == 0) and np.all(rep_y == 0) and np.all(node_y == 0)

    if is_2d:
        # 2D Plot
        fig, ax = plt.subplots(figsize=(12, 8))

        # Plot nodes with cost colorbar
        scatter = ax.scatter(node_x, node_z, c=node_cost, cmap='viridis', s=30, label='Nodes')

        # Plot witnesses as white circles with black borders
        for i in range(len(witness_x)):
            ax.scatter(witness_x[i], witness_z[i], color='white', edgecolor='black', s=50, zorder=5)

        # Plot representatives in red
        ax.scatter(rep_x, rep_z, c='red', s=70, label='Representatives', zorder=6)

        # Plot connections to representatives
        for i in range(len(witness_x)):
            if not pd.isna(rep_x[i]):
                ax.plot(
                    [witness_x[i], rep_x[i]],
                    [witness_z[i], rep_z[i]],
                    c='gray', linestyle='--'
                )

        # Add circles for coverage radius around representatives
        for i in range(len(rep_x)):
            if not pd.isna(rep_x[i]):
                circle = plt.Circle((rep_x[i], rep_z[i]), radius, color='red', fill=False, linestyle='--', alpha=0.5)
                ax.add_artist(circle)

        # Add colorbar
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('Node Cost')

        # Labels and legend
        ax.set_title("2D Witness Set with Nodes and Coverage Radius (-Z)", fontsize=16)
        ax.set_xlabel("X-axis")
        ax.set_ylabel("-Z-axis")
        ax.legend()

    else:
        # 3D Plot
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Plot nodes with cost colorbar
        scatter = ax.scatter(node_x, node_y, node_z, c=node_cost, cmap='viridis', s=30, label='Nodes')

        # Plot witnesses as white circles with black borders
        ax.scatter(witness_x, witness_y, witness_z, facecolors='white', edgecolors='black', s=50, label='Witnesses', zorder=5)

        # Plot representatives in red
        ax.scatter(rep_x, rep_y, rep_z, c='red', s=70, label='Representatives', zorder=6)

        # Plot connections to representatives
        for i in range(len(witness_x)):
            if not pd.isna(rep_x[i]):
                ax.plot(
                    [witness_x[i], rep_x[i]],
                    [witness_y[i], rep_y[i]],
                    [witness_z[i], rep_z[i]],
                    c='gray', linestyle='--'
                )

        # Add spheres for coverage radius around representatives
        u = np.linspace(0, 2 * np.pi, 50)
        v = np.linspace(0, np.pi, 50)
        for i in range(len(rep_x)):
            if not pd.isna(rep_x[i]):
                x_sphere = radius * np.outer(np.cos(u), np.sin(v)) + rep_x[i]
                y_sphere = radius * np.outer(np.sin(u), np.sin(v)) + rep_y[i]
                z_sphere = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + rep_z[i]
                ax.plot_wireframe(x_sphere, y_sphere, z_sphere, color='red', alpha=0.2)

        # Add colorbar
        cbar = plt.colorbar(scatter, ax=ax, pad=0.1)
        cbar.set_label('Node Cost')

        # Labels and legend
        ax.set_title("3D Witness Set with Nodes and Coverage Radius (-Z)", fontsize=16)
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("-Z-axis")
        ax.legend()

    # Show plot
    plt.show()

plot_graph('tree.csv')
plot_witness_set_with_nodes('witness_set.csv', 'tree.csv', 1)