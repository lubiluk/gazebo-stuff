### IROS 2018 Gazebo models and plugins
# Skill Transfer

## Installation

1. Clone the repo anywhere in your file system, (e.g. ~/) 
   ```
   git clone https://github.com/lubiluk/iros2018-gazebo
   ```

2. Edit your bashrc file and add the following Gazebo paths:
   ```
   export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/iros2018-gazebo/plugins/build:/opt/ros/kinetic/lib
   export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/iros2018-gazebo/models
   ```
   This will make the plugins and models available in Gazebo.
   You can now open Gazebo and see if you can insert models onto the stage.

3. Compile plugins:
   ```
   cd ~/iros2018-gazebo/plugins
   mkdir build
   cd build
   cmake ..
   make
   ```
