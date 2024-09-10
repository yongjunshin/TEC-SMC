import numpy as np

def normal_latency(mean, stddev):
    latency = np.random.normal(mean, stddev, 1)[0]
    if latency < 0:
        latency = 0
    return latency