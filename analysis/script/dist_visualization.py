import numpy as np
import matplotlib.pyplot as plt
import csv

# Define the PDF function
def normal_pdf(x, mean, std_dev):
    coefficient = 1 / (std_dev * np.sqrt(2 * np.pi))
    exponent = np.exp(-0.5 * ((x - mean) / std_dev) ** 2)
    return coefficient * exponent


# Function to plot the histogram and normal distribution for each state
def plot_state_distribution(state_name, samples, mean, stddev):
    # Convert the list of samples to a numpy array
    samples = np.array(samples)
    
    # Create a range of values for the normal distribution curve
    min_sample, max_sample = np.min(samples), np.max(samples)
    x_values = np.linspace(min_sample, max_sample, 1000)
    
    # Calculate the normal distribution curve using numpy
    normal_dist = normal_pdf(x_values, mean, stddev)
    # (1 / (stddev * np.sqrt(2 * np.pi))) * np.exp(-0.5 * (((x_values - mean) / stddev) ** 2))
    print(sum(normal_dist))
    print(max(normal_dist))
    # Normalize the normal distribution to match the histogram

    # Create a figure and axis
    fig, ax1 = plt.subplots(figsize=(9, 6))
    
    # Plot histogram with density=True
    hist_values, bins, _ = ax1.hist(samples, bins=30, density=True, alpha=0.6, color='b', label='Sample Histogram')
    
    # Plot the normal distribution
    ax2 = ax1.twinx()
    ax2.plot(x_values, normal_dist, 'r-', lw=2, label='Normal Distribution')
    
    # Set labels and titles
    ax1.set_xlabel('Staying time (second)')
    ax1.set_ylabel('Number of samples')
    ax1.set_title(state_name)
    
    # Set the limits of the y-axes
    ax1.set_ylim(bottom=0)
    ax2.set_ylim(bottom=0)
    ax2.set_yticks([])
    
    # Adjust layout to ensure legends fit outside the plot area
    fig.subplots_adjust(right=0.7)  # Adjust right side of the plot to make room for legends
    
    # Combine legends and place them outside the upper right corner
    ax1.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)
    ax2.legend(loc='upper left', bbox_to_anchor=(1.05, 0.92), borderaxespad=0.)
    
    # Show the plot
    plt.show()

# Function to read CSV and visualize the data
def visualize_from_csv(filename):
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            state = row['State']
            mean = float(row['Mean'])
            stddev = float(row['StdDev'])
            
            # Collect all sample values into a list
            samples = [float(row[f'Sample{i+1}']) for i in range(len(row) - 3) if row[f'Sample{i+1}']]
            
            # Plot the distribution and histogram
            plot_state_distribution(state, samples, mean, stddev)


# Example usage
csv_filename = 'analysis/results/log_anal.csv'  # Replace with your actual CSV file
visualize_from_csv(csv_filename)
