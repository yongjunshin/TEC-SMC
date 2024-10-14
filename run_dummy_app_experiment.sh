#!/bin/bash

# Define colors
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

# Check if any parameter is empty or if incorrect number of arguments is provided
if [ -z "$1" ] || [ -z "$2" ] || [ -z "$3" ]; then
    echo -e "${RED}Usage: $0 <config> <num_experiments> <time_bound>${NC}"
    exit 1
fi

config=$1
num_experiments=$2
time_bound=$3

check_ros2_nodes() {
    local nodes=$(ros2 node list)
    if [ -z "$nodes" ]; then
        # echo "All ROS2 nodes have been successfully terminated."
        return 0
    else
        echo -e "${RED}WARNING: Some ROS2 nodes are still running${NC}:"
        echo "$nodes"
        return 1
    fi
}

echo -e "${GREEN}Start experiments.${NC}"
echo

# Step 1
echo -e "${GREEN}Step 1: Run ROS application and collect logs.${NC}"

source ./cases/dummy_app/install/local_setup.bash
sleep 1

for idx in $(seq 1 $num_experiments); do
    log_file="analysis/dummy_app/logs/dummy_app_${config}_${idx}_log.txt"
    exec ros2 launch bringing_up my_TEC_SMC_bringing_up.launch.py > "$log_file" 2>&1 &
    sleep 1

    # ps -o pid,pgid,sid,cmd -e | grep -E "ros2|my_"
    # ros2 node list
    # pgrep -n -f "ros2 launch bringing_up"

    ros_pid=$(pgrep -n -f "ros2 launch bringing_up")
    ros_pgid=$(ps -o pgid= -p $ros_pid | tr -d ' ')
    # echo "$ros_pid"
    # echo "$ros_pgid"

    # Wait for the specified time
    echo "$idx th experiment (pid:$ros_pid($ros_pgid)) running for $time_bound seconds..."
    sleep "$time_bound"

    # echo "Stopping experiment $idx with SIGINT..."
    # echo "kill -2 $ros_pgid"
    kill -2 "$ros_pid"
    # echo "Rest for cleanup"
    sleep 5

    # ps -o pid,pgid,sid,cmd -e | grep -E "ros2|my_"
    # ros2 node list
    check_ros2_nodes
    # echo
done
echo

# Step 2
echo -e "${GREEN}Step 2: Run TE_profiler.${NC}"
python3 analysis/tools/TE_profiler.py -logDir analysis/dummy_app/logs -appName dummy_app -config "$config" -numLog "$num_experiments" -resultDir analysis/dummy_app/results
echo 

# Step 3
echo -e "${GREEN}Step 3: Run UPPAAL profile generation${NC}"
python3 analysis/tools/uppaal_code_generation/dummy_app_uppaal_profile_generation.py \
    -template analysis/tools/uppaal_code_generation/dummy_app_uppaal_profile_template.c \
    -tprofile "analysis/dummy_app/results/dummy_app_${config}_Tprofile.csv" \
    -eprofile "analysis/dummy_app/results/dummy_app_${config}_Eprofile.csv" \
    -powerDur 0.1 \
    -config "${config}"
echo

# Step 4
echo -e "${GREEN}Step 4: Run statistical verification (test) of ROS logs${NC}"
python3 analysis/tools/statistical_verification.py -config "analysis/tools/vconfigs/dummy_${config}_vconf.yaml"
echo

echo -e "${GREEN}Experiments completed.${NC}"
echo "Now, go to UPPAAL to run SMC, and compare the SMC results with the results of this experiment."