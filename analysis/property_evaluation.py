import re

def checker_p1(file_contents, args):
    # cold-start latency (Qual)
    cold_start_latency = checker_p3(file_contents, args)
    cold_start_eval_result = cold_start_latency < args.cold_latency

    return cold_start_eval_result

def checker_p2(file_contents, args):
    # cold-start latency (Quan, P)
    return checker_p1(file_contents, args)

def checker_p3(file_contents, args):
    # cold-start latency (Quan, CI)
    starting_time = sensor_sense_start_time(file_contents, args.sensor)
    end_time = actuator_subscribe_end_time(file_contents, args.actuator)
    cold_start_latency = end_time - starting_time

    return cold_start_latency

def checker_p4(file_contents, args):
    # latency check (Qual)
    max_durations = checker_p6(file_contents, args)
    max_latency_eval_result = max_durations < args.latency

    return max_latency_eval_result

def checker_p5(file_contents, args):
    # latency check (Quan, P)
    return checker_p4(file_contents, args)

def checker_p6(file_contents, args):
    # latency check (Quan, CI)
    durations = collect_duration(file_contents, args.actuator, "Subscribe")
    max_durations = max(durations[1:])  # maximum except the first cold-start duration

    return max_durations

def checker_p7(file_contents, args):
    # TODO: Implement evaluation logic for p7
    return True

def checker_p8(file_contents, args):
    # TODO: Implement evaluation logic for p8
    return True

def checker_p9(file_contents, args):
    # TODO: Implement evaluation logic for p9
    return True

def checker_p10(file_contents, args):
    # TODO: Implement evaluation logic for p10
    return True

def checker_p11(file_contents, args):
    # TODO: Implement evaluation logic for p11
    return True

def checker_p12(file_contents, args):
    # TODO: Implement evaluation logic for p12
    return True


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
        r'\[.*?\] \[INFO\] \[\d+\.\d+\] \[' + re.escape(node_name) + r'\]: ' + 
        re.escape(state_name) + r' state \(end\) \(' + re.escape(state_name) + 
        r' duration:(\d+\.\d+)\).*'
    )

    # Find all matches in the file contents
    matches = pattern.findall(file_contents)

    # Convert matched strings to floats and add to durations list
    durations = [float(duration) for duration in matches]

    return durations