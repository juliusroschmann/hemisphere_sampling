# Viewing Hemisphere Sampling
This repository contains the python files to sample view points from a sphere based on an icosahedron.

## Execution:
Run in your console ```foo@bar:~$ .../path_to_folder/ python3 hemisphere_sampling_tracebot.py```

### File: settings_tracebot.py
Set needed parameters in this file. All adaptations should be made here at first.

### File: ico_tracebot.py
Logic to create icosahedron with the desired number of view points. Do not manipulate this file.

### File: hemisphere_sampling_tracebot.py
ROS-based logic to move the robot to each generated view point. Visualize the topics in RVIZ set in the settings file to see the view points and which view point is approached next.
