# Introduction

## Demo:



[![Alt text](https://github.com/haxhimitsu/iksolver_track/blob/master/readme_material/preview.png)](https://www.youtube.com/watch?v=Cliyr5ubmo0)


## Dependencies
* Ubuntu 16.04, 18.04, 20.04
* ROS kinetic, melodic, noetic
* [universal_robot package](https://github.com/naoteen/universal_robot)

## Installation
~~~
cd ~/catkin_ws/src
git clone https://github.com/naoteen/iksolver_trac.git
sudo apt install libnlopt-dev 
sudo apt install swig
sudo apt install ros-noetic-nlopt
sudo apt install libnlopt-cxx-dev
<!-- 
rosdep install -i --from-paths path-to-ros-package
-->
cd ~/catkin_ws
catkin_make
source devel/setup.bash
~~~

## How to use it
### Cartesian pose controller

```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
roslaunch trac_ik_examples tracik.launch sim:=true
rosrun trac_ik_examples takler_IK_target_pose.py
```

* input : target frame(pose,orientation) through topic "IK_target_pose"
* output : each targt joit potision

``track_ik_example/src/takler_IK_target_pose.py`` is main control script. You can use it as guide to write your code.\
You can choose the IK solver, trackIK or moveit. Please commentout one of them for your purpose.
```
"choose the solver, trackIK or moveit"
# # trackIK
# def setPose(self, pose):
#     if self.checkSafety(pose):
#         ps = PoseStamped()
#         ps.pose = pose
#         self.pub.publish(ps)
#         self.pre_pose = pose

# moveit
def setPose(self, pose):
    if self.checkSafety(pose):
        self.mani.set_pose_target(pose)
        _, plan, _, _ = self.mani.plan()
        self.mani.execute(plan)
```

If you use real robot, ``sim:=false``


<!-- ### for position and force controller (not available now)
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
       -->
      
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
