from pyJoules.device import DeviceFactory
from pyJoules.device.rapl_device import RaplPackageDomain, RaplDramDomain
from pyJoules.device.nvidia_device import NvidiaGPUDomain
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
    print(sample.timestamp, sample.tag, sample.duration, sample.energy)
