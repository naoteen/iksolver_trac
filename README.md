# Introduction

## Demo:



[![Alt text](https://github.com/haxhimitsu/iksolver_track/blob/master/readme_material/preview.png)](https://www.youtube.com/watch?v=Cliyr5ubmo0)


## Dependencies
* Ubuntu 18.04, 20.04
* ROS melodic
* [universal_robot](https://github.com/naoteen/universal_robot)
## Installation
~~~
cd ~/catkin_ws/src
git clone https://github.com/naoteen/iksolver_trac.git
sudo apt-get install libnlopt-cxx-dev swig ros-noetic-nlopt
rosdep install -i --from-paths path-to-ros-package
cd ~/catkin_ws
catkin_make
source devel/setup.bash
~~~

## Launch file
* Cartesian position controller.
  * input : target frame(pose,orientation) through topic "IK_target_pose"
  * output : each targt joit potision solved track_ik
```
roslaunch trac_ik_examples tracik.launch sim:=false
```
if you use gazebo simulation, sim:=true


realtime ik solver node is``trac_ik_jointpub.cpp``


### for position and force controller
 * send target frame and force.
   * msg_type:``std_msgs_Float32MultiArray``
   * send to :```\array```
    ```
    rostopic pub  -r 500 /array std_msgs/Float32MultiArray "layout:
    dim:
    - label: ''
        size: 10
        stride: 0
    data_offset: 0
    data:
    - 0.56
    - 0.028
    - 0.611
    - 0.0
    - 0.0
    - 0.0
    - 1.0
    - 0.0
    - 0.0
    - 0.00
    "
    ```
    and run
    ```
    rosrun trac_ik_examples pos_force_controller
    ```
    finally  run ik_solver
    ```
    rosrun trac_ik_examples trac_ik_jointpub
    ```

*  Note : each valuable set as
      ```
        rostopic pub  -r 500 /array std_msgs/Float32MultiArray "layout:
        dim:
        - label: ''
            size: 10
            stride: 0
        data_offset: 0
        data:
        - pose.x
        - pose.y
        - pose.z
        - rotation.x
        - rotation.y
        - rotation.z
        - rotation.w
        - force.x
        - force.y
        - force.z
        "
      ```
      force ranges limited -9<value<9 at ```pos_force_controller.cpp```
      
      
## How to set joint limit

if you use ```roslaunch ur_gazebo ur5.launch limited:=true```, IK provide multiple result.
For this reason, Robot may be vibrate.

One of the most simple solution is  editing  joint limits at ```ur5_joint_limited_robot.urdf.xacro```.

```
  <!-- arm -->
  <xacro:ur5_robot prefix="" joint_limited="true"
    shoulder_pan_lower_limit="${-pi/4.0}" shoulder_pan_upper_limit="${pi/4.0}"
    shoulder_lift_lower_limit="${-pi}" shoulder_lift_upper_limit="${pi*0.0}"
    elbow_joint_lower_limit="${-pi*0.0}" elbow_joint_upper_limit="${pi}"
    wrist_1_lower_limit="${-pi*1.3}" wrist_1_upper_limit="${-pi/3.0}"
    wrist_2_lower_limit="${-pi}" wrist_2_upper_limit="${pi*0.0}"
    wrist_3_lower_limit="${-pi}" wrist_3_upper_limit="${pi}"
    transmission_hw_interface="$(arg transmission_hw_interface)"
  />
```


  ## Reference
* This repository reference [trac_ik](https://bitbucket.org/traclabs/trac_ik/src/master/)
