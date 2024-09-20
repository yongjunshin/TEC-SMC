import os
import sys
import argparse
from property_evaluation import (
    checker_p1, checker_p2, checker_p3, checker_p4, checker_p5, checker_p6,
    checker_p7, checker_p8, checker_p9, checker_p10, checker_p11, checker_p12
)


def parse_arguments():
    parser = argparse.ArgumentParser(description="SMC Verification Program")
    
    parser.add_argument("-numSample", type=int, default=100, help="Number of samples for SMC (default: 100)")
    parser.add_argument("-timeBound", type=float, default=10, help="Time bound for SMC (default: 10)")
    parser.add_argument("-confLevel", type=float, default=0.95, help="Confidence level for SMC (default: 0.95)")
    parser.add_argument("-latency", type=float, required=True, help="Upper bound of latency requirement")
    parser.add_argument("-cold_latency", type=float, required=True, help="Upper bound of cold-start latency requirement")
    parser.add_argument("-energy", type=float, required=True, help="Upper bound of energy consumption requirement")
    parser.add_argument("-cost", type=float, required=True, help="Upper bound of infra usage cost requirement")
    parser.add_argument("-appName", type=str, required=True, help="Application name")
    parser.add_argument("-config", type=str, required=True, help="Configuration ID")
    parser.add_argument("-logDir", type=str, required=True, help="Directory path of ROS logs")
    parser.add_argument("-sensor", type=str, required=True, help="Name of sensor node")
    parser.add_argument("-actuator", type=str, required=True, help="Name of actuator node")

    return parser.parse_args()


def print_parameters(args):
    print("Parameter Settings:")
    print(f"  SMC Number of Samples: {args.numSample}")
    print(f"  SMC Time Bound: {args.timeBound}")
    print(f"  SMC Confidence Level: {args.confLevel}")
    print(f"  Latency Requirement: {args.latency if args.latency is not None else 'Not set'}")
    print(f"  Latency Requirement: {args.cold_latency if args.cold_latency is not None else 'Not set'}")
    print(f"  Energy Requirement: {args.energy if args.energy is not None else 'Not set'}")
    print(f"  Cost Requirement: {args.cost if args.cost is not None else 'Not set'}")
    print(f"  Application Name: {args.appName}")
    print(f"  Configuration ID: {args.config}")
    print(f"  Log Directory: {args.logDir}")
    print(f"  Sensor Node: {args.sensor}")
    print(f"  Actuator Node: {args.actuator}")
    print()


def check_log_files(log_dir, app_name, config, num_samples):
    for trial_num in range(1, num_samples + 1):
        filename = f"{app_name}_{config}_{trial_num}.txt"
        if not os.path.exists(os.path.join(log_dir, filename)):
            print(f"Error: Missing log file {filename}")
            sys.exit(1)
    print(f"All {num_samples} log files found.")


def evaluate_sample(sample_log_filename, args):
    try:
        with open(sample_log_filename, 'r') as file:
            file_contents = file.read()
            
        print(f"Evaluating {sample_log_filename}")
        
        evaluation_results = {
            "p1": checker_p1(file_contents, args),
            "p2": checker_p2(file_contents, args),
            "p3": checker_p3(file_contents, args),
            "p4": checker_p4(file_contents, args),
            "p5": checker_p5(file_contents, args),
            "p6": checker_p6(file_contents, args),
            "p7": checker_p7(file_contents, args),
            "p8": checker_p8(file_contents, args),
            "p9": checker_p9(file_contents, args),
            "p10": checker_p10(file_contents, args),
            "p11": checker_p11(file_contents, args),
            "p12": checker_p12(file_contents, args)
        }
        
        return {"filename": sample_log_filename, "results": evaluation_results}
    except FileNotFoundError:
        print(f"Error: File {sample_log_filename} not found.")
        return {"filename": sample_log_filename, "status": "file_not_found"}
    except Exception as e:
        print(f"Error reading {sample_log_filename}: {str(e)}")
        return {"filename": sample_log_filename, "status": "error"}


def verify_samples(evaluation_result_list):
    # TODO: Implement the statistical verification logic
    # Return statisticalVerificationResult
    for sample in evaluation_result_list:
        print(sample['filename'], sample['results'])


def save_verification_result(verification_result):
    # TODO: Implement the logic to save the verification result
    print("Verification Result:")
    pass


def main():
    args = parse_arguments()

    # Print all parameter settings
    print_parameters(args)

    # Access parameters using args.parameter_name
    check_log_files(args.logDir, args.appName, args.config, args.numSample)
    
    evaluation_result_list = []
    
    for trial_num in range(1, args.numSample + 1):
        sample_log_filename = f"{args.appName}_{args.config}_{trial_num}.txt"
        sample_log_path = os.path.join(args.logDir, sample_log_filename)
        
        sample_evaluation_result = evaluate_sample(sample_log_path, args)
        evaluation_result_list.append(sample_evaluation_result)
    
    statistical_verification_result = verify_samples(evaluation_result_list)
    save_verification_result(statistical_verification_result)


if __name__ == "__main__":
    main()