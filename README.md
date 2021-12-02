# simulation_sound_play


# Run the simulation

You can run simulation as follows: 

```
roslaunch simulation_sound_play trajectory_following.launch
```

Launch keyboard controller: 

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

Start sound trajectory tracking in `.../catkin_ws/src/simulation_sound_play/scripts`: 
```
./trajectory_tracking_sound.py 
```
 
# Docker

Use pulseaudio enabled Docker to enable access to sound devices. 
You can find it [here](https://github.com/larics/docker_files/tree/master/ros-melodic/mmuav_audio_ros) 

# Research possibilities 

Explore possibilities of collision detection/avoidance based on sound change. 

[Doppler effect](https://en.wikipedia.org/wiki/Doppler_effect) is interesting thing which could be simulated depending on UAV movement. 

Sound generation with python can be found [here](https://stackoverflow.com/questions/9770073/sound-generation-synthesis-with-python) and [here](https://github.com/luvsound/pippi). 

Idea is to use safety radius when certain object enters it we generate sound which corresponds to source of that object. 
Check source sound generation and theoretical background for sound generation. 

Constrict on 1 obstacle and different safety levels of UAV. 


# TODO: 
 - [ ] Add trajectory drawings 
 - [ ] Check why UAV climbs at all times
 - [ ] Check different distances maybe 
 - [ ] Check how to use EAST/NORTH/WEST/SOUTH convention 
 - [ ] Define critical height below which UAV says 'GO UP TO PREVENT FALL' 
 
