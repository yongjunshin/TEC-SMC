import re
import csv
import numpy as np
from collections import defaultdict
from scipy.stats import norm

# Function to extract state information from log lines
def extract_state_info(line):
    # #match = re.search(r'\[(\d+\.\d+)\].*:(\w+ state) \((start|end)\)', line)
    # match = re.search(r'\[(\d+\.\d+)\]\s*\[(\w+)\]\s*:\s*(\w+ state) \((start|end)\)', line)
    # if match:
    #     return float(match.group(1)), match.group(2)+' '+match.group(3), match.group(4)

    match = re.search(r'\[(\d+\.\d+)\]\s*\[(\w+)\]\s*:\s*(\w+ state) \((start|end)\)(?:\s*\([\w\s]+ power:\s*([\d.]+)\))?', line)
    if match:
        timestamp = float(match.group(1))
        component_state = match.group(2) + ' ' + match.group(3)
        start_end = match.group(4)
        power = float(match.group(5)) if match.group(5) else None
        return timestamp, component_state, start_end, power
    
    return None

# Read the log file and process state information
def process_log(filename):
    states = defaultdict(list)
    current_states = {}

    with open(filename, 'r') as file:
        for line in file:
            info = extract_state_info(line)
            if info:
                timestamp, state, action, power = info
                if action == 'start':
                    current_states[state] = timestamp
                elif action == 'end' and state in current_states:
                    duration = timestamp - current_states[state]
                    states[state].append(duration)
                    states[state + ' power'].append(power)
                    del current_states[state]

    return states

# Calculate statistics for each state
def calculate_stats(states):
    stats = {}
    for state, durations in states.items():
        mean = np.mean(durations)
        std = np.std(durations)

        # Calculate z-score for 99% confidence interval
        z_score = norm.ppf(0.995)  # 0.995 because it's a two-tailed test
        
        # Determine the thresholds for the 95% confidence interval
        lower_threshold = mean - z_score * std
        upper_threshold = mean + z_score * std
        
        # Filter out outliers
        filtered_durations = [x for x in durations if lower_threshold <= x <= upper_threshold]
        
        # Recalculate mean and std deviation with filtered data
        if filtered_durations:  # Check if there are any values left after filtering
            filtered_array = np.array(filtered_durations)
            mean_filtered = np.mean(filtered_array)
            std_filtered = np.std(filtered_array)
        else:
            # Handle the case where no data remains after filtering
            mean_filtered = mean
            std_filtered = std

        stats[state] = {'mean': mean_filtered, 'std': std_filtered, 'samples': durations}

    return stats

# Write results to CSV file
def write_csv(stats, filename):
    # Find the maximum number of samples across all states
    max_samples = max(len(data['samples']) for data in stats.values())
    
    # Create the header with dynamic Sample names
    header = ['State', 'Mean', 'StdDev'] + [f'Sample{i+1}' for i in range(max_samples)]

    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)

        for state, data in stats.items():
            print(state)
            padded_samples = data['samples'] + [''] * (max_samples - len(data['samples']))
            row = [state, data['mean'], data['std']] + padded_samples
            writer.writerow(row)

# Main execution
log_dir = '/home/yjshin/Desktop/dev/TEC-SMC/analysis/dummy_app/data/'
log_name = 'system_total_energy_1_log'
result_dir = '/home/yjshin/Desktop/dev/TEC-SMC/analysis/dummy_app/results/'

log_filename = log_dir + log_name + '.txt'
csv_filename = result_dir + log_name + '_anal.csv'

# Process the log file
states = process_log(log_filename)

# Calculate statistics
stats = calculate_stats(states)

# Write results to CSV
write_csv(stats, csv_filename)

print(f"Analysis complete. Results written to {csv_filename}")