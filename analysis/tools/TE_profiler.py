import os
import sys
import argparse
import yaml
import sys
import numpy as np
import math
import re
import csv



def parse_arguments():
    parser = argparse.ArgumentParser(description="SMC Verification Program")
    parser.add_argument("-logDir", type=str, required=True, help="Path to the log file directory")
    parser.add_argument("-appName", type=str, required=True, help="Application name")
    parser.add_argument("-config", type=str, required=True, help="Configuration ID of the application")
    parser.add_argument("-numLog", type=int, required=True, help="Number of logs")
    parser.add_argument("-outlierPercentail", type=int, default=1, help="Percentail of outlier")
    parser.add_argument("-resultDir", type=str, required=True, help="Path to the profiling result file directory")



    return parser.parse_args()


def print_parameters(args):
    print("Verification Configuration")
    for arg, value in vars(args).items():
        print(f"{arg}: {value}")
    print()


def extract_state_info(line):
    # Updated regex pattern based on the given line structure
    match = re.search(
        r'\[\w+-\d+\]\s\[INFO\]\s\[\d+\.\d+\]\s\[(\w+)\]:\s(\w+)\sstate\s\(end\)\s'
        r'\(\w+\s+duration:([\d.]+)\)\s'
        r'\(\w+\s+power:([\d.]+)\)\s'
        r'\(\w+\s+energy:([\d.]+)\)', 
        line
    )

    if match:
        node = match.group(1)               # Extract the node (e.g., my_sensor)
        state = match.group(2)              # Extract the state (e.g., Sense)
        duration = float(match.group(3))    # Extract the duration (e.g., 0.020106)
        power = float(match.group(4))       # Extract the power (e.g., 0.094568)
        energy = float(match.group(5))      # Extract the energy (e.g., 0.001901)
        return node, state, duration, power, energy

    return None


def outlier_extraction(samples, outlier_percentail):    
    # Calculate 1st and 99th percentiles
    lower_threshold = np.percentile(samples, outlier_percentail)
    upper_threshold = np.percentile(samples, 100 - outlier_percentail)
    
    # Filter out outliers
    filtered_samples = samples[(samples >= lower_threshold) & (samples <= upper_threshold)]

    return filtered_samples


def write_time_profile_csv(latency_profile, latency_samples, filename):
    # Find the maximum number of samples across all states
    max_samples = max(len(latency_samples[node][state]) for node in latency_samples for state in latency_samples[node])

    # Create the header with dynamic Sample names
    header = ['Node', 'State', 'Mean', 'StdDev'] + [f'Sample{i+1}' for i in range(max_samples)]

    print("Time profile:", filename)
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)

        for node in latency_profile.keys():
            for state in latency_profile[node].keys():
                mean, std = latency_profile[node][state]
                samples_list = latency_samples[node][state].tolist()
                padding_size = max_samples - len(samples_list)
                padded_samples = samples_list + [''] * padding_size
                row = [node, state, mean, std] + padded_samples
                print([node, state, mean, std])
                writer.writerow(row)

def write_energy_profile_csv(power_profile, power_samples, filename):
    header = ['Node', 'State', 'Mean', 'StdDev'] + [f'Sample{i+1}' for i in range(len(power_samples))]

    print("Energy profile:", filename)
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)

        mean, std = power_profile
        row = ['System', 'Power', mean, std] + power_samples.tolist()
        print([mean, std])
        writer.writerow(row)


def main():
    args = parse_arguments()
    print_parameters(args)

    num_logs = args.numLog

    total_power_samples = []
    latency_samples = {}
    latency_profiles = {}

    # data extraction
    print("Log files")
    for i in range(num_logs):
        log_idx = i + 1
        file_name = f"{args.logDir}/{args.appName}_{args.config}_{log_idx}_log.txt"
        print(file_name)
        try:
            with open(file_name, 'r') as file:
                for line in file:
                    info = extract_state_info(line)
                    if info:
                        node, state, latency, power, energy = info
                        
                        if not (node in latency_samples):
                            latency_samples[node] = {}
                            latency_profiles[node] = {}
                        
                        if not (state in latency_samples[node]):
                            latency_samples[node][state] = []
                        
                        latency_samples[node][state].append(latency)
                            
                        total_power_samples.append(power)

        except FileNotFoundError:
            print(f"Error: File {file_name} not found.")
            return None
        except Exception as e:
            print(f"Error reading {file_name}: {str(e)}")
            return None
    print()

    # latency profiling
    for node in latency_samples.keys():
        for state in latency_samples[node].keys():
            latency_samples[node][state] = outlier_extraction(np.array(latency_samples[node][state]), args.outlierPercentail)
            latency_mean = np.mean(latency_samples[node][state])
            latency_std = np.std(latency_samples[node][state])
            latency_profiles[node][state] = (latency_mean, latency_std)

    time_profile_file_name = f"{args.resultDir}/{args.appName}_{args.config}_Tprofile.csv"
    write_time_profile_csv(latency_profiles, latency_samples, time_profile_file_name)

    # power profiling
    total_power_samples_array = outlier_extraction(np.array(total_power_samples), args.outlierPercentail)
    power_profile = (np.mean(total_power_samples_array), np.std(total_power_samples_array))
    print()

    time_profile_file_name = f"{args.resultDir}/{args.appName}_{args.config}_Eprofile.csv"
    write_energy_profile_csv(power_profile, total_power_samples_array, time_profile_file_name)



if __name__ == "__main__":
    main()