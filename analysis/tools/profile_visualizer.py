import numpy as np
import matplotlib.pyplot as plt
import csv
import argparse
from scipy.stats import norm

def plot_profile(data):
    node = data[0]
    state = data[1]
    mean = float(data[2])
    stddev = float(data[3])
    samples = [float(value) for value in data[4:] if value]

    # Create a histogram of the samples
    plt.figure(figsize=(10, 6))
    #plt.hist(samples, bins=20, color='blue', alpha=0.7, label='samples', density=False)
    _, bins, _ = plt.hist(samples, bins=20, color='blue', alpha=0.7, label='samples', density=False)

    # Create a normal distribution based on the mean and standard deviation
    x = np.linspace(mean - 4*stddev, mean + 4*stddev, 100)
    y = norm.pdf(x, mean, stddev)
    y = norm.pdf(x, mean, stddev) * (bins[1] - bins[0]) * len(samples)  # Scale the normal distribution
    plt.plot(x, y, color='red', label='profile')

    # Set the title and labels
    plt.title(f"{node} {state}")
    plt.ylabel("numSamples")
    plt.legend(loc='upper right')
    
    # Show the plot
    plt.tight_layout()
    plt.show()

def main():
    parser = argparse.ArgumentParser(description='Profile Visualizer')
    parser.add_argument('-profile', required=True, help='Path to the CSV profile file.')
    args = parser.parse_args()

    with open(args.profile, 'r') as csvfile:
        reader = csv.reader(csvfile)
        header = next(reader)  # Skip the header row
        for row in reader:
            plot_profile(row)

if __name__ == '__main__':
    main()