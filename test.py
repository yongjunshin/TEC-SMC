from pyJoules.device import DeviceFactory
from pyJoules.energy_meter import EnergyMeter

devices = DeviceFactory.create_devices()
meter = EnergyMeter(devices)

def foo():
    for i in range(100):
        pass

def bar():
    for i in range(1000000000):
        pass

meter.start(tag='foo')
# meter.record(tag='foo')
foo()
meter.record(tag='bar')
# meter.record(tag='bar')
bar()
meter.stop()

trace = meter.get_trace()

for sample in trace:
    total_energy = 0
    print(sample.timestamp, sample.tag, sample.duration, sample.energy)
    total_energy = sum(sample.energy.values())
    total_power = total_energy / sample.duration
    print('total_energy:', total_energy, total_power)

print('total_energy:', sum(trace[0].energy.values())/trace[0].duration)
