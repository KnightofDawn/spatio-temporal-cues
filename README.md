# spatio-temporal-cues

Current packages:
- simple_speech_interface: simple python interface to Google asr used to translate speech to text
- prolog_interface: swi-prolog wrapper needed to implement the grammars to pars the text commands
- simple_pnp_action_server: interface to Petri Net Plans

Additional packages needed:
- Petri Net Plans available at: https://github.com/iocchi/PetriNetPlans. 
- strands_morse: https://github.com/g-gemignani/strands_morse



## Requirements

Install dependencies with something like this:

`rosdep install --from-paths spatio-temporal-cues  --ignore-src`

Update scipy to version > 0.14 by running:

`sudo pip install -U scipy`


## Spatial Model

Run with:
```bash
# set this the path where your mongodb is
export DB_PATH=/Users/nah/code/datacentres/sapienza/
roslaunch mongodb_store mongodb_store.launch db_path:=$DB_PATH
# launch the simulator
roslaunch strands_morse sapienza_all.launch
# launch soma and the spatial models
roslaunch gmm_spatial_model spatial-model.launch
# launch the pnp_action_server
roslaunch simple_pnp_action_server robot_plan.launch 
# run the robot interface, only input "deliver book nick"
roslaunch simple_speech_interface prolog_interface.launch
```

Visualise with:
```bash
rviz -d `rospack find gmm_spatial_model`/spatial-cues.rviz
```

Trigger a spatial model generation with

```bash
# rosrun gmm_spatial_model spatial_model_client.py near <id of soma object>
rosrun gmm_spatial_model spatial_model_client.py near 2
```


# Datasets

## University of Birmingham, School of Computer Science, First Floor

### Adding Data

1. Start mongodb as above
2. ``` cd `rospack find gmm_spatial_model`/../data/bham_cs_1 ```
3. `mongorestore   -h localhost --port 62345 dump`

This adds data to the soma and soma_roi message_store collections. There is not yet a geospatial store for this data, but we can make one if necessary.

### Viewing Data

You can view the data in rviz as follows. 

1. Start mongodb as above
1. ``` rosrun map_server map_server `rospack find gmm_spatial_model`/../data/bham_cs_1/map/cs_1f_20140724.yaml ```
2. `rosrun soma_manager soma.py 1f_20140724 default`
3. `rosrun soma_roi_manager soma_roi.py 1f_20140724 default`
4. ``` rviz -d `rospack find gmm_spatial_model`/spatial-cues.rviz ```



