# Robotics Engineering Internship Task Level-1
This repository consists of two ROS Packages ```sensors``` and ```data_processor```. The ```sensors``` package consists of an executable named ```sensors``` which can start 3 publisher nodes for three sensors with their respective rates of publishing publishing random values. The ```data_processor``` package consists of an executable named ```data_processor``` which can start 3 subscriber nodes for three sensors with their respective rates of publishing publishing random values.

This repository has been created in ```Ros Foxy``` and on ```Ubuntu 20.04.1 LTS```. The programming language used through out this repository is ```C++```. ```Python``` is used for plotting the avergae data.. Make sure to have ```matplotlib``` and ```pandas``` libraries for plotting.

## Steps to Run the Packages
Extract the repository to the ```src``` folder of your ros2 workspace.
In a new terminal

### Run without launch file
```
# Source your ROS2 installation if need be
colcon build --packages-select sensors data_processor
# In a new terminal run the publishers
ros2 run sensors sensors
# In a new terminal run the subscribers
ros2 run data_processor data_processor
```

### Run with launch file
```
# Source your ROS2 installation if need be
colcon build --packages-select sensors data_processor
# Go to the location of launch file in the repository
# It should be <your_ws --> sensors/launch directory>
cd src/kick_project/sensors/launch
ros2 launch all_nodes.launch
```
### Run Visualizations
To get visualizations run these steps after running the launch file
```
# Ensure your current folder has the following files
ls
# OP
all_nodes.launch  plots.py        temp_save.csv
laser_save.csv    speed_save.csv
python3 plots.py
```