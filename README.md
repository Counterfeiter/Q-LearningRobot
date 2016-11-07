Artificial Neural Network and reinforcement learning (Q-Learning) walking robot C-Project
==============

Thats an example with an robot "dog" with just 4 servo motors.
The HC-SR04 will measure the distance to the wall. 
An ANN will be trained by an Q-Learning algorithm.
reward: distance to the wall gets smaller

Hardware
--------------
STM32F4 Discovery Board -> CortexM4 with FPU

License
--------------
GPLv2

Libs
--------------
fann (LGPL see http://leenissen.dk/) (added as static library to this project)
STM32 HAL

Software
--------------
System Workbench (Eclipse based)
STM CubeMX

Introduction
--------------
Press the blue Discovery button and the learnphase is starting.
If the distance to the wall gets to small or to large at the training phase the robot starts flashing an LED. Put the robot in the middle of the training stage and press the button to go on.
After the learn phase all LEDs goes on. Press blue button again to execute the learned ann.