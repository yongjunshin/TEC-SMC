import re
import csv
import numpy as np
from collections import defaultdict
from scipy.stats import norm
import scipy.stats as stats

# Function to extract state information from log lines
def extract_state_info(line):
    # #match = re.search(r'\[(\d+\.\d+)\].*:(\w+ state) \((start|end)\)', line)
    # match = re.search(r'\[(\d+\.\d+)\]\s*\[(\w+)\]\s*:\s*(\w+ state) \((start|end)\)', line)
    # if match:
    #     return float(match.group(1)), match.group(2)+' '+match.group(3), match.group(4)

    #match = re.search(r'\[(\d+\.\d+)\]\s*\[(\w+)\]\s*:\s*(\w+ state) \((start|end)\)(?:\s*\([\w\s]+ power:\s*([\d.]+)\))?', line)
    match = re.search(r'\[(\d+\.\d+)\]\s*\[(\w+)\]\s*:\s*(\w+ state) \((start|end)\)(?:\s*\([\w\s]+ duration:\s*([\d.]+)\))?(?:\s*\([\w\s]+ power:\s*([\d.]+)\))?(?:\s*\([\w\s]+ energy:\s*([\d.]+)\))?', line)
    if match:
        timestamp = float(match.group(1))
        component_state = match.group(2) + ' ' + match.group(3)
        start_end = match.group(4)
        duration = float(match.group(5)) if match.group(5) else None
        power = float(match.group(6)) if match.group(6) else None
        energy = float(match.group(7)) if match.group(7) else None
        return timestamp, component_state, start_end, duration, power, energy
    
    return None

# Read the log file and process state information
def process_log(filename):
    states = defaultdict(list)
    current_states = {}

    with open(filename, 'r') as file:
        for line in file:
            info = extract_state_info(line)
            if info:
                timestamp, state, action, dur, power, energy = info
                if action == 'start':
                    current_states[state] = timestamp
                elif action == 'end' and state in current_states:
                    duration = timestamp - current_states[state]
                    states[state].append(duration)
                    states[state + ' duration'].append(dur)
                    states[state + ' power'].append(power)
                    states[state + ' energy'].append(energy)
                    del current_states[state]

    return states

def fit_and_test(data, distributions):
    results = []

    # Convert to numpy array for easier calculations
    data_array = np.array(data)
    
    # Calculate 1st and 99th percentiles
    lower_threshold = np.percentile(data_array, 1)
    upper_threshold = np.percentile(data_array, 99)
    
    # Filter out outliers
    filtered_data = data_array[(data_array >= lower_threshold) & (data_array <= upper_threshold)]

    for dist_name in distributions:
        dist = getattr(stats, dist_name)
        params = dist.fit(filtered_data)
        
        # Perform Kolmogorov-Smirnov test
        ks_statistic, p_value = stats.kstest(filtered_data, dist_name, args=params)
        
        results.append({
            'distribution': dist_name,
            'parameters': params,
            'ks_statistic': ks_statistic,
            'p_value': p_value,
            'dist_param': dist
        })
    
    return sorted(results, key=lambda x: x['ks_statistic'])

# Calculate statistics for each state
def calculate_stats(states, dist_to_test):
    stats = {}
    for state, durations in states.items():
        print(state)
        results = fit_and_test(durations, dist_to_test)

        # Find the result with the minimum p-value
        min_ks_value_result = min(results, key=lambda x: x['ks_statistic'])

        # Print the result with the minimum p-value
        print(f"Distribution: {min_ks_value_result['distribution']}")
        print(f"KS statistic: {min_ks_value_result['ks_statistic']:.4f}")
        print(f"p-value: {min_ks_value_result['p_value']:.8f}")
        print(f"Parameters: {min_ks_value_result['parameters']}")
        print(f"Param: {min_ks_value_result['dist_param']}")
        print()

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
log_name = 'system_total_0_log'
result_dir = '/home/yjshin/Desktop/dev/TEC-SMC/analysis/dummy_app/results/'

log_filename = log_dir + log_name + '.txt'
csv_filename = result_dir + log_name + '_anal.csv'

# Process the log file
states = process_log(log_filename)

# Calculate statistics
distributions_to_test = ['norm', 'gamma', 'beta', 'arcsine', 'triang', 'weibull_min']
stats = calculate_stats(states, distributions_to_test)

# Write results to CSV
# write_csv(stats, csv_filename)

# print(f"Analysis complete. Results written to {csv_filename}")