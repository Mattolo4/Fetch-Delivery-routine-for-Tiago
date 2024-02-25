
# Fetch-Delivery Routine for Tiago 

The goal of the project is to implement a fetch and delivery behavior for the robot Tiago. 

All the details are specified in the **Report.pdf**
## Run Locally  

Clone the package 

~~~bash  
  git clone https://github.com/Mattolo4/Fetch-Delivery-routine-for-Tiago.git
~~~

Go to the workspace that contains the package

~~~bash  
  cd catkin_ws
~~~

To start the simulation, launch the simulation with all action server nodes and service nodes. 

~~~bash  
roslaunch assignment_2_group_12 Assignment2.launch
~~~

Start the **node_a**  
(NB. wait until Tiago's arm is tucked before launching node_a)

~~~bash  
rosrun assignment_2_group_12 node_a
~~~
Starting the main node the pick and place procedure will start.  

## Developers
- [Gabriel Taormina](mailto:gabriel.taormina@protonmail.com) 
- Stefano Trenti
- [Matteo Villani](https://github.com/Mattolo4?tab=repositories)




[![MIT License](https://img.shields.io/badge/License-MIT-green.svg)](https://choosealicense.com/licenses/mit/)  