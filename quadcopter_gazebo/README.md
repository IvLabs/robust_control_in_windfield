## Installation Instructions

#### Install google protocol buffer messaging library and glog library
```
sudo apt-get install libgflags-dev
sudo apt install libgoogle-glog-dev
sudo apt-get install protobuf-compiler libprotobuf-dev
```
* Clone in <your_catkin_ws>/src
* Compile (use preferably catkin build)

#### ROS Topic for publishing actuator commands:
* '/desired/motor_speeds'    
* How to publish msgs? Message Type? :- refer to testing.py for example
