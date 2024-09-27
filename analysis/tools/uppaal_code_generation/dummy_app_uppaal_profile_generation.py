import argparse
import csv
import os

def read_template(template_path):
    """Read the template C file and return its content."""
    with open(template_path, 'r') as file:
        return file.read()

def write_output(output_path, content):
    """Write the modified content to the output file."""
    with open(output_path, 'w') as file:
        file.write(content)

def replace_variables(template, variables):
    """Replace placeholders in the template with actual values from the variables dictionary."""
    for key, value in variables.items():
        # print(key, value, type(value))
        # print('{'+key+'}')
        if isinstance(value, float):
            value = round(value, 6)
        template = template.replace('{'+key+'}', str(value))
    return template

def read_profile_csv(file_path):
    """Read the T or E profile CSV file and return a dictionary of Node_State -> (Mean, StdDev)"""
    profile_data = {}
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        header = next(reader)  # Skip the header row
        for row in reader:
            node = row[0]
            state = row[1]
            mean = float(row[2]) if row[2] else 0.0
            stddev = float(row[3]) if row[3] else 0.0
            profile_data[f"{node}_{state}"] = (mean, stddev)
    return profile_data

def map_variables(tprofile_data, eprofile_data, args):
    """Map the values from the profiles to the template variables."""
    variables = {}

    # Power profile (from eprofile)
    power_key = "System_Power"
    if power_key in eprofile_data:
        variables['power_mean'], variables['power_std'] = eprofile_data[power_key]
    else:
        variables['power_mean'], variables['power_std'] = 0.0, 0.0

    # Power duration
    variables['power_duration'] = args.powerDur

    # Mapping for time profiles
    time_mappings = {
        'sensor_sense': 'sensor_Sense',
        'localization_subscribe': 'localization_Subscribe',
        'localization_processing': 'localization_Processing',
        'localization_preprocessing': 'localization_Preprocessing',
        'localization_wait': 'localization_Wait',
        'localization_postprocessing': 'localization_Postprocessing',
        'perception_subscribe': 'perception_Subscribe',
        'perception_processing': 'perception_Processing',
        'perception_preprocessing': 'perception_Preprocessing',
        'perception_wait': 'perception_Wait',
        'perception_postprocessing': 'perception_Postprocessing',
        'planning_subscribe': 'planning_Subscribe',
        'planning_processing': 'planning_Processing',
        'planning_preprocessing': 'planning_Preprocessing',
        'planning_wait': 'planning_Wait',
        'planning_postprocessing': 'planning_Postprocessing',
        'control_subscribe': 'control_Subscribe',
        'control_processing': 'control_Processing',
        'control_preprocessing': 'control_Preprocessing',
        'control_wait': 'control_Wait',
        'control_postprocessing': 'control_Postprocessing',
        'vehicle_interface_subscribe': 'vehicle_interface_Subscribe',
    }

    # Mapping each time profile key to its mean and std
    for var_name, node_state in time_mappings.items():
        mapped_key_mean = f"{var_name}_mean"
        mapped_key_std = f"{var_name}_std"
        # Find matching node in tprofile
        for profile_key in tprofile_data:
            if node_state in profile_key:
                variables[mapped_key_mean], variables[mapped_key_std] = tprofile_data[profile_key]
                break
        else:
            # If not found, set default 0.0
            variables[mapped_key_mean], variables[mapped_key_std] = 0.0, 0.0

    # Split flags (set to true if there is a "wait" state for the corresponding node)
    nodes_with_wait_states = {
        'localization_split_flag': 'my_localization',
        'perception_split_flag': 'my_perception',
        'planning_split_flag': 'my_planning',
        'control_split_flag': 'my_control'
    }

    for flag_var, node in nodes_with_wait_states.items():
        # Check if there's any entry in the tprofile data with the "wait" state for the node
        wait_key = f"{node}_wait"
        variables[flag_var] = 'true' if any(wait_key in profile_key for profile_key in tprofile_data) else 'false'

    return variables

def main():
    # Argument parsing
    parser = argparse.ArgumentParser(description='UPPAAL Profile Generation Script')
    parser.add_argument('-template', required=True, help='Path to the template C file.')
    parser.add_argument('-tprofile', required=True, help='Path to the T profile CSV file.')
    parser.add_argument('-eprofile', required=True, help='Path to the E profile CSV file.')
    parser.add_argument('-powerDur', type=float, required=True, help='Power duration value.')
    parser.add_argument('-config', required=True, help='Profile configuration.')

    
    args = parser.parse_args()

    # Read the profile CSV files
    tprofile_data = read_profile_csv(args.tprofile)
    # print(tprofile_data)
    eprofile_data = read_profile_csv(args.eprofile)
    # print(eprofile_data)

    # Read the template file
    template_content = read_template(args.template)

    # Map the values from the profiles to the template variables
    variables = map_variables(tprofile_data, eprofile_data, args)
    # print(variables)

    # Replace the variables in the template with actual values
    modified_content = replace_variables(template_content, variables)

    # Write the modified content to an output file
    output_path = os.path.splitext(args.template)[0] + '_' + args.config + '_generated.c'  # Create output filename
    write_output(output_path, modified_content)

    print(f"Profile generation complete. Output written to: {output_path}")

if __name__ == '__main__':
    main()
