# Baxter-Pick-and-Place-using-Q-Learning
Baxter robot has been trained to group similar colored blocks together and then stack them using Q-Learning

IMPORTANT PREREQUISITES:

1. Ubuntu 14.04-  http://howtoubuntu.org/how-to-install-ubuntu-14-04-trusty-tahr
2. ROS Indigo - http://wiki.ros.org/indigo/Installation/Ubuntu
3. Installing OpenCV 3.0- https://docs.opencv.org/3.0-beta/doc/tutorials/introduction/linux_install/linux_install.html
4. Baxter SDK- http://sdk.rethinkrobotics.com/wiki/Baxter_Simulator
5. MATLAB- https://www.mathworks.com/help/install/ug/install-mathworks-software.html

TRAINING:

Baxter_Training_2.mat contains the Q-Table of the trained agent. Use Player.mat to check the training. 
Player.mat provides the agent with a random sequence of numbers to sort. The color of the blocks on the Baxter are mapped to numbers in an array to simplify the learning process.

color_3, color_2, color_1 represent the colors that appear 3,2,1 times respectively. color_3, color_2, color_1 mat files store arrays that contain the colors for each of the 60 possible states. Their use can be seen in Baxter_Training_2.mat.

The codes are straightforward and implement Q-Learning using a Q-Table. 
The actions are chosen by the epsilon-greedy algorithm. The training is done for all possible states out of the total 4^6 states. 

