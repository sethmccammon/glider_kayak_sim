#!/bin/bash


roslaunch glider_kayak_sim world_launch.launch &
pid=$!

sleep 5s

for IDX in $(seq 1 $(rosparam get num_gliders));
do
	echo $IDX
	roslaunch glider_kayak_sim glider_launch.launch glider_idx:=$IDX &
	pid="$! $pid"
done

for IDX in $(seq 1 $(rosparam get num_kayaks));
do
	echo $IDX
	roslaunch glider_kayak_sim kayak_launch.launch kayak_idx:=$IDX &
	pid="$! $pid"
done

echo $pid
trap "echo Killing all processes.; kill -s TERM $pid; exit" SIGINT SIGTERM
sleep 1000d