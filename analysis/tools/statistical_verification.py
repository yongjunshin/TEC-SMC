import os
import sys
import argparse
import yaml
import sys
import numpy as np
import math
from property_checker import (
    get_cold_start_latency, get_max_latency, get_total_energy_consumption, get_total_infra_cost,
    checker_p1, checker_p2, checker_p3, checker_p4, checker_p5, checker_p6,
    checker_p7, checker_p8, checker_p9, checker_p10, checker_p11, checker_p12
)


def load_config(config_file):
    with open(config_file, 'r') as file:
        return yaml.safe_load(file)

def parse_arguments():
    parser = argparse.ArgumentParser(description="SMC Verification Program")
    parser.add_argument("-config", type=str, required=True, help="Path to the YAML configuration file")
    return parser.parse_args()

def validate_config(config):
    # Define the expected parameters with their types and requirements
    param_specs = {
        "numSample": {"type": int, "default": 100},
        "timeBound": {"type": float, "default": 10},
        "confLevel": {"type": float, "default": 0.95},
        "latency": {"type": float, "required": True},
        "coldLatency": {"type": float, "required": True},
        "energy": {"type": float, "required": True},
        "cost": {"type": float, "required": True},
        "appName": {"type": str, "required": True},
        "config": {"type": str, "required": True},
        "logDir": {"type": str, "required": True},
        "sensor": {"type": str, "required": True},
        "actuator": {"type": str, "required": True},
        "powerMean": {"type": float, "required": True},
        "powerStd": {"type": float, "required": True},
        "powerProfilingDur": {"type": float, "required": True},
        "resultSave": {"type": bool, "required": True},
        "resultSaveDir": {"type": str, "required": True}
    }

    validated_config = {}

    for param, spec in param_specs.items():
        if param in config:
            if not isinstance(config[param], spec["type"]):
                print(f"Error: {param} must be of type {spec['type'].__name__}")
                sys.exit(1)
            validated_config[param] = config[param]
        elif "required" in spec and spec["required"]:
            print(f"Error: {param} is required but not provided in the config file")
            sys.exit(1)
        elif "default" in spec:
            validated_config[param] = spec["default"]
        else:
            print(f"Warning: {param} is not provided and has no default value")

    validated_args = argparse.Namespace(**validated_config)
    return validated_args


def print_parameters(args):
    print("Verification Configuration")
    for arg, value in vars(args).items():
        print(f"{arg}: {value}")
    print()


def check_log_files(log_dir, app_name, config, num_samples):
    for trial_num in range(1, num_samples + 1):
        filename = f"{app_name}_{config}_{trial_num}_log.txt"
        if not os.path.exists(os.path.join(log_dir, filename)):
            print(f"Error: Missing log file {filename}")
            sys.exit(1)
    print(f"All {num_samples} log files found.")


def evaluate_sample(sample_log_filename, args):
    try:
        with open(sample_log_filename, 'r') as file:
            file_contents = file.read()
        
        cold_start_latency = get_cold_start_latency(file_contents, args)
        max_latency = get_max_latency(file_contents, args)
        total_energy = get_total_energy_consumption(file_contents, args)
        total_cost = get_total_infra_cost(file_contents, args)

        evaluation_results = {
            "p1": checker_p1(cold_start_latency, args.coldLatency),
            "p2": checker_p2(cold_start_latency, args.coldLatency),
            "p3": checker_p3(cold_start_latency),
            "p4": checker_p4(max_latency, args.latency),
            "p5": checker_p5(max_latency, args.latency),
            "p6": checker_p6(max_latency),
            "p7": checker_p7(total_energy, args.energy),
            "p8": checker_p8(total_energy, args.energy),
            "p9": checker_p9(total_energy),
            "p10": checker_p10(total_cost, args.cost),
            "p11": checker_p11(total_cost, args.cost),
            "p12": checker_p12(total_cost)
        }
        # print(f"{sample_log_filename}: {evaluation_results}")
        
        return evaluation_results
    except FileNotFoundError:
        print(f"Error: File {sample_log_filename} not found.")
        return None
    except Exception as e:
        print(f"Error reading {sample_log_filename}: {str(e)}")
        return None


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
    epsilon = math.sqrt(math.log(2 / delta) / (2 * n)) * (b - a)
    
    # Calculate the confidence interval
    lower_bound = p_hat - epsilon
    upper_bound = p_hat + epsilon
    
    return p_hat, lower_bound, upper_bound, epsilon


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
    samples = {}
    
    for key, values in property_results.items():
        # Convert the list of values to a numpy array and handle boolean values
        samples[key] = values
        np_values = np.array(values)

        # if np_values.dtype == 'bool':  # Convert boolean values to integers
        #     np_values = np_values.astype(int)
        
        # Compute the confidence interval using Chernoff-Hoeffding bound
        p_hat, lower_bound, upper_bound, epsilon = chernoff_hoeffding_confidence_interval(np_values, confidence_level=conf_level)
        
        # Store the result in the dictionary
        confidence_intervals[key] = (p_hat, lower_bound, upper_bound, epsilon)
    
    return confidence_intervals, samples


def save_verification_result(verification_result, prop_eval_samples, args):
    # Initialize an empty list to collect the output lines
    output_lines = []

    # Iterate over the verification results
    for property_name, (p_hat, lower_bound, upper_bound, epsilon) in verification_result.items():        
        if property_name in ['p1', 'p4', 'p7', 'p10']:
            # For properties p1, p4, p7, p10, check against confidence level
            result = p_hat > args.confLevel
            output_lines.append(f"{property_name}: {result}")
        
        elif property_name in ['p2', 'p5', 'p8', 'p11']:
            # For properties p2, p5, p8, p11, calculate mean of bounds
            mean_value = (lower_bound + upper_bound) / 2
            output_lines.append(f"{property_name}: {mean_value:.6f}")
        
        elif property_name in ['p3', 'p6', 'p9', 'p12']:
            # For properties p3, p6, p9, p12, print bounds
            output_lines.append(f"{property_name}: ({lower_bound:.6f}, {upper_bound:.6f})")

    # Print all results
    print("Statistical verification results")
    for line in output_lines:
        print(line)
    print()

    # Iterate over the verification results
    for property_name, samples in prop_eval_samples.items():        
        output_lines.append(f"{property_name} samples: {samples}")

    # Save the results to a file if specified
    if args.resultSave:
        result_file_path = f"{args.resultSaveDir}/{args.appName}_{args.config}_verif.txt"
        with open(result_file_path, 'w') as result_file:
            for line in output_lines:
                result_file.write(line + '\n')
        print(f"Verification results are saved in {result_file_path}.")


def main():
    args = parse_arguments()
    config = load_config(args.config)
    args = validate_config(config)

    # Print all parameter settings
    print_parameters(args)

    # Access parameters using args.parameter_name
    check_log_files(args.logDir, args.appName, args.config, args.numSample)
    
    evaluation_result_list = []
    
    # print("Log sample property evaluation results")
    for trial_num in range(1, args.numSample + 1):
        sample_log_filename = f"{args.appName}_{args.config}_{trial_num}_log.txt"
        sample_log_path = os.path.join(args.logDir, sample_log_filename)
        
        sample_evaluation_result = evaluate_sample(sample_log_path, args)
        evaluation_result_list.append(sample_evaluation_result)
    print()

    statistical_verification_result, prop_eval_samples = verify_samples(evaluation_result_list, args.confLevel)
    save_verification_result(statistical_verification_result, prop_eval_samples, args)


if __name__ == "__main__":
    main()