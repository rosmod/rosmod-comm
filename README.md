## ROSMOD Communication Layer

* The ROSMOD Communication Layer is a modified version of the ros_comm communication layer in ROS. 
* ROSMOD introduces PFIFO and EDF-based scheduling schemes to the ROS callback queue.
* ROSMOD also facilitates deadline monitoring i.e. deadline violation detection and logging for all callbacks.

### Dependencies

Install these dependencies according to their install instructions.

* [ROS](http://www.ros.org)
* [Catkin Tools](https://github.com/catkin/catkin_tools)

### Configuration

```bash
# chown /opt for the user
$ sudo chown -R $USER /opt
# set rosmod to extend your ROS workspace
$ catkin config --extend /opt/ros/kinetic
# set the install location for rosmod (you can change this to be wherever you like)
$ catkin config -i /opt/rosmod
# configure rosmod to actually install
$ catkin config --install
# Remove any previous build files that may exist
$ catkin clean -b --yes
```

### Build and Install

```bash
$ catkin build
```
