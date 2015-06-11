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
