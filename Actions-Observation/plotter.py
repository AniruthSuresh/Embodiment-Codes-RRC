import numpy as np
import matplotlib.pyplot as plt
import os

def plot_joint_data():
    """
    Loads action and observation data from text files and plots a comparison
    for each joint, saving the plots to a new directory.
    """
    try:
        # --- 1. Load the data from .txt files ---
        print("Loading data from actions.txt and observations.txt...")
        actions = np.loadtxt("actions.txt")
        observations = np.loadtxt("observations.txt")
        print("Data loaded successfully.")

        # Basic validation
        if actions.shape[1] < 7 or observations.shape[1] < 7:
            print(f"Error: Expected at least 7 columns, but found {actions.shape[1]} for actions and {observations.shape[1]} for observations.")
            return

        # --- 2. Create a directory for the plots ---
        plot_dir = "joint_plots"
        if not os.path.exists(plot_dir):
            os.makedirs(plot_dir)
            print(f"Created directory: {plot_dir}")
        else:
            print(f"Directory '{plot_dir}' already exists. Files will be overwritten.")

        # --- 3. Generate and save a plot for each joint ---
        num_joints_to_plot = 7
        timesteps = np.arange(actions.shape[0])

        for i in range(num_joints_to_plot):
            plt.figure(figsize=(10, 6))
            
            # Plot commanded action (from actions.txt)
            plt.plot(timesteps, actions[:, i], label=f'Action (Commanded Position)', alpha=0.8)
            
            # Plot resulting observation (from observations.txt)
            # We only take the first 7 columns of observation, which are the positions
            plt.plot(timesteps, observations[:, i], label=f'Observation (Actual Position)', alpha=0.8)
            
            # Add plot details
            plt.title(f'Joint {i+1} Position: Action vs. Observation')
            plt.xlabel('Timestep')
            plt.ylabel('Position (radians)')
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            
            # Save the figure
            save_path = os.path.join(plot_dir, f'joint_{i+1}_comparison.png')
            plt.savefig(save_path)
            plt.close() # Close the figure to free up memory
            print(f"Saved plot: {save_path}")

        print("\nAll plots have been generated and saved.")

    except FileNotFoundError:
        print("Error: 'actions.txt' or 'observations.txt' not found.")
        print("Please make sure you have run the previous script to generate these files.")
    except Exception as e:
        print(f"An error occurred: {e}")

# Run the plotting function
plot_joint_data()