def get_power(meter):
    sample = meter.get_trace()[0]
    duration = sample.duration  # seconds (s)
    energy = sum(sample.energy.values())    # micro Joules (uJ)
    energy = energy * 0.000000001  # killo Joules (kJ)
    power = energy/duration     # killo Watt (kW)
    return sample.tag, duration, power, energy   # ms, kW, kJ