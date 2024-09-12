def get_power(meter):
    sample = meter.get_trace()[0]
    duration = sample.duration  # seconds (s)
    duration = duration * 1000  # milli seconds (ms)
    energy = sum(sample.energy.values())    # micro Joules (uJ)
    energy = energy * 0.000001  # Joules (J)
    power = energy/duration     # killo Watt (kW)
    return sample.tag, duration, power, energy   # ms, kW, J