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
                # if action == 'start':
                #     current_states[state] = timestamp
                # elif action == 'end' and state in current_states:
                #     duration = timestamp - current_states[state]
                #     states[state].append(duration)
                #     states[state + ' duration'].append(dur)
                #     states[state + ' power'].append(power)
                #     states[state + ' energy'].append(energy)
                #     del current_states[state]
                
                    
                if action == 'end':
                    if (dur is None):
                        print(dur)
                    if (power is None):
                        print(power)
                    if (energy is None):
                        print(energy)
                    states[state + ' duration'].append(dur)
                    states[state + ' power'].append(power)
                    states[state + ' energy'].append(energy)
    return states

def calculate_stats(states):
    stats = {}
    for name, samples in states.items():
        # Convert to numpy array for easier calculations
        sample_array = np.array(samples)
        
        # Calculate initial mean and std
        mean = np.mean(sample_array)
        std = np.std(sample_array)
        
        # Calculate 1st and 99th percentiles
        lower_threshold = np.percentile(sample_array, 1)
        upper_threshold = np.percentile(sample_array, 99)
        
        # Filter out outliers
        filtered_durations = sample_array[(sample_array >= lower_threshold) & 
                                             (sample_array <= upper_threshold)]
        
        # Recalculate mean and std deviation with filtered data
        if filtered_durations.size > 0:  # Check if there are any values left after filtering
            mean_filtered = np.mean(filtered_durations)
            std_filtered = np.std(filtered_durations)
        else:
            # Handle the case where no data remains after filtering
            mean_filtered = mean
            std_filtered = std

        stats[name] = {
            'mean': round(mean_filtered, 6), 
            'std': round(std_filtered, 6), 
            'samples': filtered_durations.tolist()  # Convert back to list for consistency
        }

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
stats = calculate_stats(states)

# Write results to CSV
write_csv(stats, csv_filename)

print(f"Analysis complete. Results written to {csv_filename}")