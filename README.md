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

