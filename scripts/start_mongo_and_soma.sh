#!/bin/bash       
export PROJECT_DIR=$(pwd) 
roslaunch mongodb_store mongodb_store.launch db_path:=$DATA_DIR/strands_sim_example &
rosrun soma_manager soma.py $PROJECT_DIR/strands_morse/sapienza/maps/map.yaml $PROJECT_DIR/soma/soma_managerconfig/default.json 

