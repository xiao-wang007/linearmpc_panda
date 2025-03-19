import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
import numpy as np
import yaml

def run(method_name):
    # Load CSV data
    script_dir = os.path.dirname(os.path.abspath(__file__))
    rel_folder_directory = f"../data/{method_name}"  # Adjust this
    folder_directory = os.path.join(script_dir, rel_folder_directory)

    # Loop through all trials and plot the data and output average metrics 
    for name in os.listdir(folder_directory):
        print(f"trial: {name}")
        # Load the Yaml and CSV data
        run_directory = folder_directory + "/" + name
        csv_file_path = run_directory + "/trajectory.csv"
        print(csv_file_path)
        yaml_file_path = run_directory + "/summary.yaml"

        df = pd.read_csv(csv_file_path)

        with open(yaml_file_path, "r") as file:
            yaml_data = yaml.safe_load(file)  # Use safe_load to avoid executing arbitrary code

        plot_joint_data(df)

        total_pos_tracking_error, total_vel_tracking_error, total_torque_grad_sum, task_length, total_cost, task_complete, robot_stopped = evaluate_individual_performance(
            df, yaml_data
        )

        print(f"Total Position Tracking Error: {total_pos_tracking_error}")
        print(f"Total Velocity Tracking Error: {total_vel_tracking_error}")
        print(f"Total torque grad sum: {total_torque_grad_sum}")
        print(f"Total cost: {total_cost}")
        print(f"Task length: {task_length}")
        print(f"Task complete: {task_complete}")
        print(f"Robot stopped: {robot_stopped}")

def plot_joint_data(df):

    # Define segments and their corresponding columns
    # segments = ["Position", "Velocity", "Acceleration", "Torque"]
    segments = ["Position", "Velocity"]
    commanded_prefix = "Commanded"
    actual_prefix = "Actual"
    num_joints = 7
    
    # Create plots for each segment
    for segment in segments:
        fig, axes = plt.subplots(num_joints, 1, figsize=(10, 12), sharex=True)
        fig.suptitle(f'{segment} Comparison', fontsize=14)
        
        for i in range(num_joints):
            commanded_col = f"{commanded_prefix} Joint {i} {segment}"
            actual_col = f"{actual_prefix} Joint {i} {segment}"
            
            if commanded_col in df.columns and actual_col in df.columns:
                axes[i].plot(df[commanded_col], label=f'Commanded {segment} {i}', linestyle='dashed')
                axes[i].plot(df[actual_col], label=f'Actual {segment} {i}', alpha=0.8)
                axes[i].set_ylabel(f'Joint {i}')
                axes[i].legend()
            else:
                print(f"Warning: Columns {commanded_col} or {actual_col} not found in CSV.")
        
        axes[-1].set_xlabel("Time Step")
        plt.tight_layout()
        plt.subplots_adjust(top=0.95)
        plt.show()

# This function needs to return the following:
# total pos error, velocity error, torque grad sum, robot stopped, task complete and total cost
def evaluate_individual_performance(df, yaml_data):
    segments = ["Position", "Velocity", "Acceleration", "Torque"]
    commanded_prefix = "Commanded"
    actual_prefix = "Actual"
    num_joints = 7

    total_pos_tracking_error = 0
    total_vel_tracking_error = 0
    total_torque_grad_sum = 0
    total_cost = yaml_data['Total cost']
    task_complete = yaml_data['Task complete']
    robot_stopped = yaml_data['Robot stopped']
    task_length = yaml_data['Task time']

    # total_cost /= task_length

    position_tracking_error = np.zeros(num_joints)
    velocity_tracking_error = np.zeros(num_joints)
    torque_grad_sum = np.zeros(num_joints)

    for i in range(num_joints):
        commanded_col = f"{commanded_prefix} Joint {i} Position"
        actual_col = f"{actual_prefix} Joint {i} Position"
        if commanded_col in df.columns and actual_col in df.columns:
            # position_tracking_error[i] = np.linalg.norm(df[commanded_col] - df[actual_col])
            position_tracking_error[i] = np.sum(np.abs(df[commanded_col] - df[actual_col]))
            total_pos_tracking_error += position_tracking_error[i]

        total_pos_tracking_error /= task_length

        commanded_col = f"{commanded_prefix} Joint {i} Velocity"
        actual_col = f"{actual_prefix} Joint {i} Velocity"
        if commanded_col in df.columns and actual_col in df.columns:
            # velocity_tracking_error[i] = np.linalg.norm(df[commanded_col] - df[actual_col])
            velocity_tracking_error[i] = np.sum(np.abs(df[commanded_col] - df[actual_col]))
            total_vel_tracking_error += velocity_tracking_error[i]

        total_vel_tracking_error /= task_length

        # Compute the summation of rate of jerk change
        commanded_col = f"{commanded_prefix} Joint {i} Torque"
        actual_col = f"{actual_prefix} Joint {i} Torque"
        if commanded_col in df.columns and actual_col in df.columns:
            torque_grad_sum[i] = np.sum(np.abs(np.gradient(df[actual_col], 2)))
            total_torque_grad_sum += torque_grad_sum[i]

        total_torque_grad_sum /= task_length

    return total_pos_tracking_error, total_vel_tracking_error, total_torque_grad_sum, task_length, total_cost, task_complete, robot_stopped


if __name__ == "__main__":
    # Get the directory of this script
    # name = "panda_torque_PD_controller_opt_time_to_start_torque_rate_limiter_low_pass_filter"   # Lowest cost
    # name = "panda_inverse_dynamics_controller_Start_at_0_no_acceleration_ff"   # Highest cost - however, very good velocity tracking??

    # name = "panda_torque_PD_controller_error_match_to_start"   # Highest pos error
    name = "panda_torque_PD_controller_Start_at_0_low_pass_filter"   # Highest vel error


    run(name)