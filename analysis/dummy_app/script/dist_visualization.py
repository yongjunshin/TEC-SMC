import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.stats import norm

def normal_pdf(x, mean, std_dev):
    coefficient = 1 / (std_dev * np.sqrt(2 * np.pi))
    exponent = np.exp(-0.5 * ((x - mean) / std_dev) ** 2)
    return coefficient * exponent

duration_dist_dict = {}
duration_x_dict = {}
mean_power_dict = {}

def plot_state_distribution(ax, state_name, samples, mean, stddev):
    samples = np.array(samples)
    
    min_sample, max_sample = np.min(samples), np.max(samples)
    

    x_values = np.linspace(min_sample, max_sample, 1000)
    normal_dist = normal_pdf(x_values, mean, stddev)

    ax.hist(samples, bins=30, density=True, alpha=0.6, color='b', label='Sample Histogram')
    ax_twin = ax.twinx()
    ax_twin.plot(x_values, normal_dist, 'r-', lw=2, label='Normal Distribution')

    if "energy" in state_name.lower():
        duration_str = state_name.replace('energy', 'duration')
        power_str = state_name.replace('energy', 'power')
        if (duration_str in duration_dist_dict) and (duration_str in duration_x_dict) and (power_str in mean_power_dict):
            ax_twin2 = ax.twinx()
            ax_twin2.plot(duration_x_dict[duration_str]*mean_power_dict[power_str], duration_dist_dict[duration_str], 'b-', lw=2, label='Prediction')
            ax_twin2.set_ylim(bottom=0)
            ax_twin2.set_yticks([])
            ax_twin2.legend(loc='upper left', bbox_to_anchor=(1.09, 0.3), borderaxespad=0.)


    if "duration" in state_name.lower():
        ax.set_xlabel('pyJoule duration (ms)')
        duration_dist_dict[state_name] = normal_dist
        duration_x_dict[state_name] = x_values
    elif "power" in state_name.lower():
        ax.set_xlabel('Power (kW)')  
        mean_power_dict[state_name] = mean
    elif "energy" in state_name.lower():
        ax.set_xlabel('Energy (J)')  
    else:
        ax.set_xlabel('Staying time (ms)')
        # duration_dist_dict[state_name + ' duration'] = normal_dist
        # duration_x_dict[state_name + ' duration'] = x_values
    ax.set_ylabel('Number of samples')
    ax.set_title(state_name)

    ax.set_ylim(bottom=0)
    ax_twin.set_ylim(bottom=0)
    ax_twin.set_yticks([])

    ax.legend(loc='upper left', bbox_to_anchor=(1.09, 1), borderaxespad=0.)
    ax_twin.legend(loc='upper left', bbox_to_anchor=(1.09, 0.65), borderaxespad=0.)
    
    mean_std_text = f'Mean: {mean:.9f}\nStdDev: {stddev:.9f}'
    ax.text(1.1, 0.2, mean_std_text, transform=ax.transAxes, ha='left', va='top', fontsize=10, bbox=dict(facecolor='white', alpha=0.8))

def visualize_from_csv(filename):
    data = []
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            state = row['State']
            mean = float(row['Mean'])
            stddev = float(row['StdDev'])
            samples = [float(row[f'Sample{i+1}']) for i in range(len(row) - 3) if row[f'Sample{i+1}']]
            data.append((state, samples, mean, stddev))

    fig, axes = plt.subplots(len(data), 1, figsize=(8, 2*len(data)), squeeze=False)

    for i, (state, samples, mean, stddev) in enumerate(data):
        print(state)
        plot_state_distribution(axes[i, 0], state, samples, mean, stddev)

    plt.tight_layout(pad=3, h_pad=10, w_pad=10)
    plt.subplots_adjust(top=(0.8+((0.02*len(data)))), right=0.65, hspace=1)
    plt.show()

# Example usage
result_dir = 'analysis/dummy_app/results/'
csv_filename = 'system_total_0_log_anal.csv'  # Replace with your actual CSV file
visualize_from_csv(result_dir + csv_filename)