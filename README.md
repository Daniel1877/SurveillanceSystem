# SurveillanceSystem

The project consists in a Surveillance System that detects de movement comparing consecutive pointclouds. It contains several funcionalities that allow to represent the interactive markers in order to get the orientation providing the desire position, another one that allow to record and stop it according to user necessity.

If the physical cameras are not available, the first step is to execute the simulated environment include in the system:

> roslaunch surveilance_system simulation.launch

Once the simulation is executed there are two options:

  1.- Execute the system with the RViz funcionality to get the camera's orientation, the node to detect the               movement, the node to record the environment if movement is detected and rqt_gui tool to represent the               camera's images:
  
      > roslaunch surveillance_system demo.launch
      
  2.- Execute the system with the plugin that allow to record and stop the recording as wish calling the recording         node, execute the node to detect movement but without record and the rqt_gui tool to represent the camera's         images and the plugin: 
  
      > roslaunch surveillance_system plugin.launch
