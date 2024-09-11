from pyJoules.device import DeviceFactory
from pyJoules.energy_meter import EnergyMeter

devices = DeviceFactory.create_devices()
meter = EnergyMeter(devices)

# def foo():
#     for i in range(100):
#         pass

# def bar():
#     for i in range(1000000000):
#         pass

# meter.start(tag='foo')
# # meter.record(tag='foo')
# foo()
# meter.record(tag='bar')
# # meter.record(tag='bar')
# bar()
# meter.stop()

# trace = meter.get_trace()

# for sample in trace:
#     total_energy = 0
#     print(sample.timestamp, sample.tag, sample.duration, sample.energy)
#     total_energy = sum(sample.energy.values())
#     total_power = total_energy / sample.duration
#     print('total_energy:', total_energy, total_power)

# print('total_energy:', sum(trace[0].energy.values())/trace[0].duration)

import time
def wait(duration):
    if duration < 0:
        raise ValueError("Duration must be non-negative")
    
    start_time = time.time()
    end_time = start_time + duration
    
    while time.time() < end_time:
        pass  # This is an empty loop that does nothing 


# def get_power(meter):
#     sample = meter.get_trace()[0]
#     duration = sample.duration # duration (second)
#     energy = sum(sample.energy.values()) # energy (joule)
#     power = energy/duration # power (watt)
#     return sample.tag, duration, power, energy

meter.start(tag='foo')

for i in range(100):
    wait(0.1)
    meter.record(tag='foo')

meter.stop()

trace = meter.get_trace()

i = 0
power_hist = []
for sample in trace:
    total_energy = 0
    print(i)
    print(sample.timestamp, sample.tag, sample.duration, sample.energy)
    total_energy = sum(sample.energy.values())
    total_power = total_energy / sample.duration
    power_hist.append(total_power)
    print('total_energy:', total_energy, ' total_power:', total_power)
    i = i + 1


import matplotlib.pyplot as plt

plt.hist(power_hist, bins=100, edgecolor='black')

# Set titles and labels
plt.title('title')
plt.xlabel('values')
plt.ylabel('frequency')

# Display the histogram
plt.show()
