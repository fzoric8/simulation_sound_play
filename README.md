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


# TODO: 
 - [ ] Add trajectory drawings 
 - [ ] Check why UAV climbs at all times
 - [ ] Check different distances maybe 
 - [ ] Check how to use EAST/NORTH/WEST/SOUTH convention 
 - [ ] Define critical height below which UAV says 'GO UP TO PREVENT FALL' 
 
