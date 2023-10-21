# Viewing Hemisphere Sampling
This repository contains the python files to sample view points from a sphere using an icosahedron.

## Execution:
Run ```console
foo@bar:~$ .../path_to_folder/ python3 hemisphere_sampling_tracebot.py```

### File: settings_tracebot.py
Set needed hemisphere parameters in this file.

### File: ico_tracebot.py
Logic to create icosahedron with the desired number of view points. 

### File: hemisphere_sampling_tracebot.py
ROS-based logic to move the robot to each generated view point.
