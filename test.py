import numpy as np

def chernoff_hoeffding_confidence_interval(samples, confidence_level=0.95):
    # Number of samples
    n = len(samples)
    
    # Calculate sample mean
    p_hat = np.mean(samples)
    
    # Determine the range of sample values
    if isinstance(samples[0], (bool, np.bool_)):  # Check if the samples are boolean
        a, b = 0, 1  # Boolean values are bounded by 0 and 1 (False = 0, True = 1)
    else:
        a, b = np.min(samples), np.max(samples)  # For floating-point values, use the min and max of the data
    
    # Confidence level (e.g., 0.95 for 95%)
    delta = 1 - confidence_level
    
    # Solve for epsilon using the inverse of the Chernoff-Hoeffding bound, scaled by the range
    epsilon = np.sqrt(np.log(2 / delta) / (2 * n)) * (b - a)
    
    # Calculate the confidence interval
    lower_bound = p_hat - epsilon
    upper_bound = p_hat + epsilon
    
    return lower_bound, upper_bound

def verify_samples(evaluation_result_list, conf_level):
    # Extract property keys
    property_keys = evaluation_result_list[0].keys()
    
    # Create a dictionary to store the results for each property
    property_results = {key: [] for key in property_keys}
    
    # Populate the property results with values from the list of dictionaries
    for result_dict in evaluation_result_list:
        for key, value in result_dict.items():
            property_results[key].append(value)
    
    # Now calculate confidence intervals for each property
    confidence_intervals = {}
    
    for key, values in property_results.items():
        # Convert the list of values to a numpy array and handle boolean values
        np_values = np.array(values)
        print(np_values.dtype)
        
        if np_values.dtype == 'bool':  # Convert boolean values to integers
            np_values = np_values.astype(int)
        
        # Compute the confidence interval using Chernoff-Hoeffding bound
        lower_bound, upper_bound = chernoff_hoeffding_confidence_interval(np_values, confidence_level=conf_level)
        
        # Store the result in the dictionary
        confidence_intervals[key] = (lower_bound, upper_bound)
    
    return confidence_intervals

# Example usage
evaluation_result_list = [
    {'p1': False, 'p2': False, 'p3': 0.40575289726257324, 'p4': True, 'p5': True, 'p6': 0.034062, 'p7': False, 'p8': False, 'p9': 2.0459671846642706, 'p10': True, 'p11': True, 'p12': 0},
    {'p1': True, 'p2': True, 'p3': 0.3619239330291748, 'p4': True, 'p5': True, 'p6': 0.032764, 'p7': True, 'p8': True, 'p9': 2.0306340152949907, 'p10': True, 'p11': True, 'p12': 0},
    {'p1': True, 'p2': True, 'p3': 0.3852860927581787, 'p4': True, 'p5': True, 'p6': 0.039632, 'p7': False, 'p8': False, 'p9': 2.0432700380319826, 'p10': True, 'p11': True, 'p12': 0},
    # Additional samples...
]

confidence_level = 0.95
confidence_intervals = verify_samples(evaluation_result_list, confidence_level)

print(confidence_intervals)
