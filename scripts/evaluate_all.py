# import code from run_performance.py
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from run_performance import run
from run_performance import evaluate_individual_performance

import yaml

from scipy import stats
from collections import namedtuple

import math

# Define a Named Tuple for better structuring
PerformanceMetric = namedtuple("PerformanceMetric", ["mean", "std_deviation", "conf_interval"])

def evaluate_all_runs():

    # Get the number of folders in the data directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    relative_path = "../saved_data_controller_performance"
    data_dir = os.path.join(script_dir, relative_path)

    data = []

    for name in os.listdir(data_dir):
        # print(name)
        method_directory = os.path.join(data_dir, name)
        # mean_pos_error, mean_vel_error, mean_torque_grad, mean_task_length, mean_task_cost, mean_task_complete, mean_robot_stopped = evaluate_method_performance(
        #     method_directory
        # )
        performance_results = evaluate_method_performance(method_directory)
        controller_name, mpc_method, torque_rate_limiter, low_pass_filter, send_ff = get_directory_testing_parameters(method_directory)

        data.append({
                    "controller": controller_name,
                    "mpc_method": mpc_method,
                    "low_pass_filter": low_pass_filter,
                    "torque_rate_limiter": torque_rate_limiter,
                    "send_ff": send_ff,
                    "pos_error": performance_results["pos_tracking_error"],
                    "vel_error": performance_results["vel_tracking_error"],
                    "torque_grad": performance_results["torque_grad_sum"],
                    "task_length": performance_results["task_length"],
                    "task_cost": performance_results["task_cost"],
                    "task_complete": performance_results["task_complete"],
                    "robot_stopped": performance_results["robot_stopped"],
                    "num_trials": performance_results["num_trials"],
                })

    df = pd.DataFrame(data)

    metrics_to_plot = ["pos_error", "vel_error", "torque_grad", "task_length", "task_cost", "task_complete", "robot_stopped"]
    # metrics_to_plot = ["mean_pos_error", "mean_vel_error", "mean_torque_grad", "mean_task_cost"]
    # Create a unique label for each combination of mpc, controller, and liw
    # df["x_label"] = df.apply(lambda row: f"{row['mpc']}\n{row['controller']}\nLIW={row['low_pass_filter']}", axis=1)
    df["x_label"] = df.apply(create_x_label, axis=1)

    create_latex_table(df, ["controller"])

    # plot_data_averaged(df, metrics_to_plot, groupings=["controller"])
    # plot_data_averaged(df, metrics_to_plot, groupings=["mpc_method"])
    # plot_data_averaged(df, metrics_to_plot, groupings=["controller", "mpc_method", "low_pass_filter", "torque_rate_limiter", "send_ff"])

    # Plot the data in different orders
    # plot_data(df, metrics_to_plot, ordering=["task_cost"])

    # plot_data(df, metrics_to_plot, ordering=["mean_pos_error", "mean_vel_error"])

    # plot_data(df, metrics_to_plot, ordering=["mean_task_cost", "mean_vel_error"])

def temp(df):
    

def create_latex_table(df, groupings=None):

    if groupings:
        df_sorted = df.groupby(groupings).mean(numeric_only=True).reset_index()

        # df_sorted = df.sort_values(by=ordering[0], key=lambda col: col.apply(lambda x: x.mean)) if ordering else df
    else:
        df_sorted = df.sort_values(by=["controller", "mpc_method", "send_ff", "low_pass_filter", "torque_rate_limiter"])

    print(df_sorted.head())

    # Create the latex table strings so i can copy paste my data into overleaf for my thesis
    for index, row in df_sorted.iterrows():
        if "controller" in row:
            if row["controller"] == "panda_torque_PD_controller":
                controller = "Torque\_PD"
            else:
                controller = "Inverse Dynamics"
        else:
            controller = "-"

        if "mpc_method" in row:
            if row["mpc_method"] == "error_match_to_start":
                mpc = "Error"
            elif row["mpc_method"] == "Start_at_0":
                mpc = "t0"
            else:
                mpc = "time"
        else:
            mpc = "-"

        if "low_pass_filter" in row:
            if row["low_pass_filter"] == True:
                LP = "X"
            else:
                LP = ""
        else:
            LP = "-"

        if "torque_rate_limiter" in row:
            if row["torque_rate_limiter"] == True:
                TRL = "X"
            else:
                TRL = ""
        else:
            TRL = "-"

        if "send_ff" in row:
            if row["send_ff"] == True:
                FF = "X"
            else:
                FF = ""
        else:
            FF = "-"

        pos_error = f"{(row['pos_error'].mean * 1000):,.2f} $\pm$ {(row['pos_error'].std_deviation * 1000):,.2f}"
        vel_error = f"{(row['vel_error'].mean * 1000):,.2f} $\pm$ {(row['vel_error'].std_deviation * 1000):,.2f}"
        torque_grad = f"{(row['torque_grad'].mean * 1000):,.2f} $\pm$ {(row['torque_grad'].std_deviation * 1000):,.2f}"
        task_cost = f"{row['task_cost'].mean:,.2f} $\pm$ {row['task_cost'].std_deviation:,.2f}"
        task_length = f"{row['task_length'].mean:,.2f} $\pm$ {row['task_length'].std_deviation:,.2f}"
        
        #"task_complete", "robot_stopped"
        task_complete = f"{row['task_complete']} / {row['num_trials']}"
        robot_stopped = f"{row['robot_stopped']} / {row['num_trials']}"
        
        # means = df_sorted[metric].apply(lambda x: x.mean)
        # ci_lowers = df_sorted[metric].apply(lambda x: x.conf_interval[0])
        # ci_uppers = df_sorted[metric].apply(lambda x: x.conf_interval[1])
        # errors = ci_uppers - means  # Upper error bars

        formatted_string = f"{controller} & {mpc} & {LP} & {TRL} & {FF} & {pos_error} & {vel_error} & {torque_grad} & {task_cost} & {task_length} & {task_complete} & {robot_stopped} \\\\"
        print(formatted_string)

def create_x_label(row):
    """
    Generates a short identifier for each configuration.
    Format: "Ctrl: X | MPC: Y | LPF: On/Off | TRL: On/Off | FF: On/Off"
    """

    lpf = "LPF-On" if row["low_pass_filter"] else "LPF-Off"
    
    ff = "FF-On" if row["send_ff"] else "FF-Off"

    if row["mpc_method"] == "error_match_to_start":
        mpc = "mpc_error"
    elif row["mpc_method"] == "Start_at_0":
        mpc = "mpc_t0"
    else:
        mpc = "mpc_time"

    ctrl = row["controller"]
    if row["controller"] == "panda_torque_PD_controller":
        ctrl = "TPD"
        trl = "TRL-On" if row["torque_rate_limiter"] else "TRL-Off"

        x_label = f"{ctrl}_{mpc}_{lpf}_{trl}_{ff}"
    else:
        ctrl = "ID"

        x_label = f"{ctrl}_{mpc}_{lpf}_{ff}"

    return x_label
    

def plot_data(df, metrics_to_plot, ordering):
    # Sort DataFrame to maintain consistent ordering
    # df_sorted = df.sort_values(by=ordering)
    # df_sorted = df.sort_values(by=ordering[0], key=lambda col: col.apply(lambda x: x.mean))
    df_sorted = df.sort_values(by=ordering[0], key=lambda col: col.apply(lambda x: x.mean)) if ordering else df


    # Create stacked subplots
    fig, axes = plt.subplots(len(metrics_to_plot), 1, figsize=(10, 12), sharex=True)

    for i, metric in enumerate(metrics_to_plot):
        ax = axes[i]
        # mean, (ci_lower, ci_upper) = df_sorted[metric]
        means = df_sorted[metric].apply(lambda x: x.mean)
        ci_lowers = df_sorted[metric].apply(lambda x: x.conf_interval[0])
        ci_uppers = df_sorted[metric].apply(lambda x: x.conf_interval[1])
        errors = ci_uppers - means  # Upper error bars

        ax.bar(df_sorted["x_label"], means, yerr=errors, capsize=5)
        # ax.set_ylabel(metric)
        ax.set_title(f"Comparison of {metric}")

    # Adjust x-axis labels to be diagonal
    plt.xticks(rotation=45, ha="right")
    plt.xlabel("MPC Method / Controller / low_pass_filter")

    plt.tight_layout()
    plt.show()

def plot_data_averaged(df, metrics_to_plot, groupings):

    # df_grouped = df.groupby(groupings).mean().reset_index()
    df_grouped = df.groupby(groupings).mean(numeric_only=True).reset_index()

    fig, axes = plt.subplots(len(metrics_to_plot), 1, figsize=(10, 12), sharex=True)

    for i, metric in enumerate(metrics_to_plot):
        print(df[metric])
        ax = axes[i]
        ax.bar(df_grouped[groupings[0]], df_grouped[metric][0])
        # ax.set_ylabel(metric)
        ax.set_title(f"Comparison of {metric}")

# Plot mean position error and mean velocity error for different controllers
    # plt.figure(figsize=(10, 5))
    # plt.bar(df_grouped[groupings[0]], df_grouped["mean_pos_error"], width=0.4, label="Mean Position Error")
    # plt.bar(df_grouped[groupings[0]], df_grouped["mean_vel_error"], width=0.4, label="Mean Velocity Error", alpha=0.7)

    # plt.xlabel("Controller")
    # plt.ylabel("Error")
    # plt.title("Comparison of Position and Velocity Errors by Controller")
    # plt.legend()
    plt.show()

    # Sort DataFrame to maintain consistent ordering
    # df_sorted = df.sort_values(by=ordering)

    # Create stacked subplots
    # fig, axes = plt.subplots(len(metrics_to_plot), 1, figsize=(10, 12), sharex=True)

    # for i, metric in enumerate(metrics_to_plot):
    #     ax = axes[i]
    #     ax.bar(df_sorted["x_label"], df_sorted[metric])
    #     # ax.set_ylabel(metric)
    #     ax.set_title(f"Comparison of {metric}")

    # # Adjust x-axis labels to be diagonal
    # plt.xticks(rotation=45, ha="right")
    # plt.xlabel("MPC Method / Controller / low_pass_filter")

    # plt.tight_layout()
    # plt.show()

def get_directory_testing_parameters(method_directory):

    yaml_filepath = method_directory + "/0/summary.yaml"
    with open(yaml_filepath, "r") as file:
        yaml_data = yaml.safe_load(file)  

    controller_name = yaml_data['Controller']

    if controller_name == "panda_torque_PD_controller":
        torque_rate_limiter = yaml_data['Torque rate limiter']
        send_ff = yaml_data['Send torque feedforward']

    else:
        torque_rate_limiter = "none"
        send_ff = yaml_data['Send acceleration feedforward']

    mpc_method = yaml_data['mpc_start_state_method']
    low_pass_filter = yaml_data['Low pass filter']

    return controller_name, mpc_method, torque_rate_limiter, low_pass_filter, send_ff

def compute_statistical_measures(data, confidence=0.95):
    """Computes mean and confidence interval for a given dataset."""
    if len(data) == 0:
        return PerformanceMetric(mean=np.nan, conf_interval=(np.nan, np.nan))

    n = len(data)
    mean = np.mean(data)
    std_dev = np.std(data, ddof=1)  # Use ddof=1 for sample standard deviation

    z = 1.96
    margin = z *  std_dev/math.sqrt(n)
    
    return PerformanceMetric(mean=mean, std_deviation = std_dev, conf_interval=margin)


def evaluate_method_performance(method_directory):

    # print("---------------------------------------------------------------------")
    # print(method_directory)

    pos_tracking_errors = []
    vel_tracking_errors = []
    total_torque_grad_sums = []
    task_lengths = []
    total_costs = []
    task_completes = []
    robot_stoppeds = []

    # Loop through all runs in the method directory
    for name in os.listdir(method_directory):
        # print(f"trial: {name}")
        # Load the Yaml and CSV data
        run_directory = os.path.join(method_directory, name)
        csv_file_path = run_directory + "/trajectory.csv"
        yaml_file_path = run_directory + "/summary.yaml"
        
        df = pd.read_csv(csv_file_path)
        with open(yaml_file_path, "r") as file:
            yaml_data = yaml.safe_load(file)  

        # call evaluate individual performance
        total_pos_tracking_error, total_vel_tracking_error, total_torque_grad_sum, task_length, total_cost, task_complete, robot_stopped = evaluate_individual_performance(
            df, yaml_data
            )

        pos_tracking_errors.append(total_pos_tracking_error)
        vel_tracking_errors.append(total_vel_tracking_error)
        total_torque_grad_sums.append(total_torque_grad_sum)
        task_lengths.append(task_length)
        total_costs.append(total_cost)
        task_completes.append(task_complete)
        robot_stoppeds.append(robot_stopped)

    # Average the data
    mean_pos_error = np.mean(pos_tracking_errors)
    mean_vel_error = np.mean(vel_tracking_errors)
    mean_torque_grad = np.mean(total_torque_grad_sums)
    mean_task_length = np.mean(task_lengths)
    mean_task_cost = np.mean(total_costs)
    mean_task_complete = np.mean(task_completes)
    mean_robot_stopped = np.mean(robot_stoppeds)

    num_completes = 0
    num_robot_stopped = 0
    for complete in task_completes:
        if complete == True:
            num_completes += 1

    for robot_stopped in robot_stoppeds:
        if robot_stopped == True:
            num_robot_stopped += 1


    performance_results = {
        "pos_tracking_error": compute_statistical_measures(pos_tracking_errors),
        "vel_tracking_error": compute_statistical_measures(vel_tracking_errors),
        "torque_grad_sum": compute_statistical_measures(total_torque_grad_sums),
        "task_length": compute_statistical_measures(task_lengths),
        "task_cost": compute_statistical_measures(total_costs),
        "task_complete": num_completes,
        "robot_stopped": num_robot_stopped,
        "num_trials": len(pos_tracking_errors),
    }

    # print(performance_results["task_cost"])

    return performance_results

if __name__ == "__main__":
    evaluate_all_runs()