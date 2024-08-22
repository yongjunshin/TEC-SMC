import re
import csv
import numpy as np
from collections import defaultdict

# Function to extract state information from log lines
def extract_state_info(line):
    match = re.search(r'\[(\d+\.\d+)\].*: (\w+ state) \((start|end)\)', line)
    if match:
        return float(match.group(1)), match.group(2), match.group(3)
    return None

# Read the log file and process state information
def process_log(filename):
    states = defaultdict(list)
    current_states = {}

    with open(filename, 'r') as file:
        for line in file:
            info = extract_state_info(line)
            if info:
                timestamp, state, action = info
                if action == 'start':
                    current_states[state] = timestamp
                elif action == 'end' and state in current_states:
                    duration = timestamp - current_states[state]
                    states[state].append(duration)
                    del current_states[state]

    return states

# Calculate statistics for each state
def calculate_stats(states):
    stats = {}
    for state, durations in states.items():
        mean = np.mean(durations)
        std = np.std(durations)
        stats[state] = {'mean': mean, 'std': std, 'samples': durations}
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
            padded_samples = data['samples'] + [''] * (max_samples - len(data['samples']))
            row = [state, data['mean'], data['std']] + padded_samples
            writer.writerow(row)

# Main execution
log_dir = '/home/yjshin/Desktop/dev/TEC-SMC/analysis/data/'
log_name = 'log'
result_dir = '/home/yjshin/Desktop/dev/TEC-SMC/analysis/results/'

log_filename = log_dir + log_name + '.txt'
csv_filename = result_dir + log_name + '_anal.csv'

# Process the log file
states = process_log(log_filename)

# Calculate statistics
stats = calculate_stats(states)

# Write results to CSV
write_csv(stats, csv_filename)

print(f"Analysis complete. Results written to {csv_filename}")