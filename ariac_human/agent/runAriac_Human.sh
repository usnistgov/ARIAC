#!/bin/bash
ros2 launch test_competitor competitor.launch.py > tmp_outp.log &
pid_ros=$!
echo "Starting Gradlew and Monitoring ROS+Jason"
./gradlew runAntag -q --console=plain &
sleep 3
ros2 topic pub /ariac/start_human std_msgs/msg/Bool '{data: true}' --once 
LOG_ROS="ros.log"
LOG_JAS="jason.log"
PIDS_ROS=$(ps aux --sort=-%cpu | grep ros | awk 'NR<21{print $2}' | tr '\n' ',' | sed 's/.$//')
first_val=$(echo "$PIDS_ROS" | cut -d',' -f1)
top -b -d 0.01 -p $PIDS_ROS | awk \
    -v cpuLog="$LOG_ROS" '
    BEGIN{count=1; minCPU=1000.0; maxCPU=0.0; sMEM=0.0; tMEM=0.0} 
    $1+0>0 {sMEM+=$10; if($9<minCPU){minCPU=$9} if($9>maxCPU){maxCPU=$9}}
    /^$/ {if (sMEM>-1) {tMEM+=sMEM; printf "CPU-Min/Max: %3d/%4d - AvgMEM: %3.1f \n", minCPU, maxCPU, tMEM/count > cpuLog; fflush(cpuLog); sMEM=0.0; count++;}}' &
top_ros=$!

PID_JAS=$(jps | grep RunLocalMAS | awk 'NR==1{print $1}')
top -b -d 0.01 -p $PID_JAS | awk \
    -v cpuLog="$LOG_JAS" '
    BEGIN{sCPU=0.0; sMEM=0.0; count=1; tCPU=0.0; tMEM=0.0} 
    $1+0>0 {sCPU+=$9; sMEM+=$10;}
    /^$/ {if (sCPU>0) {tCPU+=sCPU; tMEM+=sMEM; printf "%d, CPU: %3d - MEM: %4.1f - AvgMEM: %3.1f \n", count, sCPU, sMEM, tMEM/count > cpuLog; fflush(cpuLog); sCPU=0.0; sMEM=0.0; count++;}}' &
top_jason=$!

end=$((SECONDS+120))
while ps -p $pid_ros > /dev/null
do
    ps -p $PID_JAS -o %cpu
    ps -p $PIDS_ROS -o %cpu | awk \
    'BEGIN{sCPU=0;} 
    $1+0>0 {sCPU+=$1; printf "\b\b\b\b%4d", sCPU;}'
    echo " "
    sleep 2
done
#echo "$(date)"
kill $top_ros   >/dev/null 2>&1
kill $top_jason >/dev/null 2>&1
touch .stop___MAS  >/dev/null 2>&1
exit 0

#Version to run until the competition launcher finishes (110s or 275s) - line 28:
#while ps -p $pid_ros > /dev/null

#Version to run for time (120s) - line 28:
#while [ $SECONDS -lt $end ]; 

# Command line test (adjust the PID):
#top -b -d 2 -p 230815 ...

# Change lines 12 and 13 for those bellow to print all values for ROS processes:
#    $1+0>0 {sCPU+=$9; printf "%d, ", $9  > cpuLog}
#    /^$/ {if (sCPU>0) {tCPU+=sCPU; printf "\b\b - Sum: %d, Avg: %f \n", sCPU, tCPU/count > cpuLog; fflush(cpuLog); sCPU=0; count++;}}' &

# Code bellow was supposed to print Min/Max CPU utilization + Memory, but is not properly summing memory. To be FIXED if required in the future
#    BEGIN{count=1; minCPU=1000.0; maxCPU=0.0; sMEM=0.0; tMEM=0.0} 
#    $1+0>0 {if($9<minCPU){minCPU=$9} if($9>maxCPU){maxCPU=$9} sMEM+=$10; printf "MEM: %3.1f - ", sMEM > cpuLog;}
#    /^$/ {if (maxCPU>-1) {tMEM+=sMEM; printf "%d CPU-Min/Max: %3d/%4d - AvgMEM: %3.1f \n", count, minCPU, maxCPU, tMEM/count > cpuLog; fflush(cpuLog); sMEM=0.0; count++;}}' &

