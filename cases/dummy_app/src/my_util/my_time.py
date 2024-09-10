import time

def wait(duration):
    if duration < 0:
        raise ValueError("Duration must be non-negative")
    
    start_time = time.time()
    end_time = start_time + duration
    
    while time.time() < end_time:
        pass  # This is an empty loop that does nothing