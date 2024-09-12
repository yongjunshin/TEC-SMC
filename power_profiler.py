from pyJoules.device import DeviceFactory
from pyJoules.energy_meter import EnergyMeter
import time
import matplotlib.pyplot as plt
import numpy as np

def wait(duration):
    if duration < 0:
        raise ValueError("Duration must be non-negative")
    
    start_time = time.time()
    end_time = start_time + duration
    
    while time.time() < end_time:
        pass  # This is an empty loop that does nothing 


def percentail_capping(samples, tail_percentail):
    # Convert to numpy array for easier calculations
    sample_array = np.array(samples)
    
    # Calculate initial mean and std
    mean = np.mean(sample_array)
    std = np.std(sample_array)
    
    # Calculate 1st and 99th percentiles
    lower_threshold = np.percentile(sample_array, tail_percentail)
    upper_threshold = np.percentile(sample_array, 100 - tail_percentail)
    
    # Filter out outliers
    filtered_samples = sample_array[(sample_array >= lower_threshold) & 
                                            (sample_array <= upper_threshold)]
    
    return filtered_samples


def normal_pdf(x, mean, std_dev):
    coefficient = 1 / (std_dev * np.sqrt(2 * np.pi))
    exponent = np.exp(-0.5 * ((x - mean) / std_dev) ** 2)
    return coefficient * exponent


# profiler user setting
profiling_duration = 0.1    # seconds
num_power_samples = 500
outlier_percentail = 5  # confidence interval = 100 - (2 * outlier_percentail)


devices = DeviceFactory.create_devices()
unit_meter = EnergyMeter(devices)
mission_meter = EnergyMeter(devices)

unit_meter.start()
mission_meter.start()
# profiling
for i in range(num_power_samples):
    time.sleep(profiling_duration)
    unit_meter.record()
unit_meter.stop()
mission_meter.stop()
trace = unit_meter.get_trace()

# power data extraction
power_hist = []
for i in range(num_power_samples):
    sample = trace[i]
    duration = sample.duration * 1000 
    energy = sum(sample.energy.values()) * 0.000001
    power = energy / duration
    power_hist.append(power)
    # print(i+1, 'th sample power: ', power, ', sample duration: ', duration)
# print('power samples: ', power_hist)

mission_sample = mission_meter.get_trace()[0]
mission_energy = sum(mission_sample.energy.values()) * 0.000001
mission_power = mission_energy / (mission_sample.duration * 1000)
print('mission power      : ', round(mission_power, 3), ', mission duration: ', round(mission_sample.duration, 3), ', mission energy: ', round(mission_energy, 3))

# analysis
hist_range_min = np.min(power_hist)
hist_range_max = np.max(power_hist)

filtered_power_hist = percentail_capping(power_hist, 1)     # outlier extraction (percentail capping)

mean_power, std_power = np.mean(filtered_power_hist), np.std(filtered_power_hist)
min_power, max_power = np.min(filtered_power_hist), np.max(filtered_power_hist)
x_values = np.linspace(min_power, max_power, 1000)
normal_dist = normal_pdf(x_values, mean_power, std_power)


# report
print('filtered power mean: ', round(np.mean(filtered_power_hist), 3))
print('filtered power std : ', round(np.std(filtered_power_hist), 3))

plt.hist(power_hist, bins=100, edgecolor='white', range=(hist_range_min, hist_range_max))   # plotting before outlier extraction
plt.hist(filtered_power_hist, bins=100, edgecolor='black', range=(hist_range_min, hist_range_max))
ax_twin = plt.twinx()
ax_twin.plot(x_values, normal_dist, 'r-', lw=2, label='Normal Distribution')

plt.title('Power profile')
plt.xlabel('Power (watt)')
plt.ylabel('Frequency')
plt.show()

