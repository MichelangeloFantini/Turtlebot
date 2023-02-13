# Turtlebot

## Description
This code was used to allow a turtebot to localize itself in a map of offices, and deliver the mail at the right office. Each of the 11 offices only characterized by one out of 4 colors. In order to properly navigate the map we used a rasberry camera for color detection, a PID controller for navigation purposes and baysian localization to extrapolate useful information from the sequance of colors seen by the camera.

The code in this repo is the principal file that could be found in the ROS workspace used to control the turtlebot from a computer.
The code takes the data from the pi camera is read, transformed in HSV, and compoared to possible color values. The color values are manually calibrated i.e. determined after some observations made by my partner and I. These colors are therefore specific to the room conditions in which we run the experiment.

Depending on the color detected by the camera, the code produces different outputs. If the color detected is black, we run the PID that allows the robot to follow the track. If the color detected is not black, then Baysian localization is performed. When the probability of being in the wanted state is higher than a threshold, the robot stops and "delivers the mail"

## Additional Info
To cope with the camera noise that occured at any color transition, we introduced a list that keeps track of the last N colors seen. If the list is full of the same color, only then a color change is registered and the output recalculated

