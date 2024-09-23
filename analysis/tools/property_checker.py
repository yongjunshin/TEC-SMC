import re
import random


def get_cold_start_latency(file_contents, args):
    starting_time = sensor_sense_start_time(file_contents, args.sensor)
    end_time = actuator_subscribe_end_time(file_contents, args.actuator)
    cold_start_latency = end_time - starting_time

    return cold_start_latency


def get_max_latency(file_contents, args):
    # latency check (Quan, CI)
    durations = collect_duration(file_contents, args.actuator, "Subscribe")
    max_latencys = max(durations[1:])  # maximum except the first cold-start duration

    return max_latencys


def get_total_energy_consumption(file_contents, args):
    power_mean = args.powerMean
    power_std = args.powerStd
    power_profiling_dur = args.powerProfilingDur
    time_bound = args.timeBound

    total_energy = 0
    elapsed_time = 0

    while elapsed_time < time_bound:
        tick_duration = min(power_profiling_dur, time_bound - elapsed_time)
        power = random.gauss(power_mean, power_std)
        tick_energy = power * tick_duration
        total_energy += tick_energy
        elapsed_time += tick_duration

    return total_energy


def get_total_infra_cost(file_contents, args):
    durations = collect_duration(file_contents, r'.*', "Wait")
    total_infra_durations = sum(durations)

    return total_infra_durations


def checker_p1(cold_start_latency, limit):
    # cold-start latency (Qual)
    return bool(cold_start_latency < limit)


def checker_p2(cold_start_latency, limit):
    # cold-start latency (Quan, P)
    return bool(cold_start_latency < limit)


def checker_p3(cold_start_latency):
    # cold-start latency (Quan, CI)
    return cold_start_latency


def checker_p4(max_latency, limit):
    # latency check (Qual)
    return bool(max_latency < limit)


def checker_p5(max_latency, limit):
    # latency check (Quan, P)
    return bool(max_latency < limit)


def checker_p6(max_latency):
    # latency check (Quan, CI)
    return max_latency


def checker_p7(total_energy, limit):
    # energy check (Qual)
    return bool(total_energy < limit)


def checker_p8(total_energy, limit):
    # energy check (Quan, P)
    return bool(total_energy < limit)


def checker_p9(total_energy):
    # energy check (Quan, CI)
    return total_energy


def checker_p10(total_cost, limit):
    # infra cost check (Qual)
    return bool(total_cost < limit)


def checker_p11(total_cost, limit):
    # infra cost check (Quan, P)
    return bool(total_cost < limit)


def checker_p12(total_cost):
    # infra cost check (Quan, CI)
    return total_cost


def sensor_sense_start_time(file_contents, sensor_node_name):
    # Regular expression to match the pattern and extract the decimal value
    pattern = r"\[\S+\] \[INFO\] \[(\d+\.\d+)\] \[" + sensor_node_name + "\]: Sense state \(start\)"
    
    # Search for the pattern in the file contents
    match = re.search(pattern, file_contents)
    
    if match:
        # Extract the decimal value from the first capturing group
        decimal_value = float(match.group(1))
        return decimal_value
    else:
        print("No match found for the given pattern.")
        return None
    

def actuator_subscribe_end_time(file_contents, actuator_node_name):
    # Regular expression to match the pattern and extract the decimal value
    pattern = r"\[\S+\] \[INFO\] \[(\d+\.\d+)\] \[" + actuator_node_name + "\]: Subscribe state \(end\)"
    
    # Search for the pattern in the file contents
    match = re.search(pattern, file_contents)
    
    if match:
        # Extract the decimal value from the first capturing group
        decimal_value = float(match.group(1))
        return decimal_value
    else:
        print("No match found for the given pattern.")
        return None
    

def collect_duration(file_contents, node_name, state_name):
    durations = []

    # Define the regex pattern
    pattern = re.compile(
        r'\[.*?\] \[INFO\] \[\d+\.\d+\] \[' + node_name + r'\]: ' + 
        state_name + r' state \(end\) \(' + state_name + 
        r' duration:(\d+\.\d+)\).*'
    )

    # Find all matches in the file contents
    matches = pattern.findall(file_contents)

    # Convert matched strings to floats and add to durations list
    durations = [float(duration) for duration in matches]

    return durations