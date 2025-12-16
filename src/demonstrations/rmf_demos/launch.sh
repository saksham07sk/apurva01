#!/bin/bash
ros2 run rmf_demos_tasks dispatch_delivery -p p1 -ph coke_dispenser -d p2 -dh coke_ingestor --use_sim_time
sleep 15

# Dispatch patrol tasks
ros2 run rmf_demos_tasks dispatch_patrol -p p5 -n 1 --use_sim_time
sleep 5

ros2 run rmf_demos_tasks dispatch_patrol -p p5 p16 -n 1 --use_sim_time
sleep 5

ros2 run rmf_demos_tasks dispatch_patrol -p p4 p10 -n 1 --use_sim_time
sleep 5

ros2 run rmf_demos_tasks dispatch_patrol -p p14 p3 -n 1 --use_sim_time
sleep 5

ros2 run rmf_demos_tasks dispatch_patrol -p p21 p14 -n 1 --use_sim_time
sleep 5

ros2 run rmf_demos_tasks dispatch_patrol -p p19 p15 -n 1 --use_sim_time
sleep 5

ros2 run rmf_demos_tasks dispatch_patrol -p p15 p16 -n 1 --use_sim_time
sleep 5


ros2 run rmf_demos_tasks dispatch_patrol -p p21 p13 -n 1 --use_sim_time
sleep 5

ros2 run rmf_demos_tasks dispatch_patrol -p p18 -n 1 --use_sim_time
sleep 5
