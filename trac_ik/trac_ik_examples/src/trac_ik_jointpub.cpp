//include KDL library
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>

//trac_ik  lib
#include <trac_ik/trac_ik.hpp>

//common
#include <stdio.h>
#include <iostream>

//ROS
 #include <ros/ros.h>
 #include <sensor_msgs/JointState.h>
 #include <std_msgs/Header.h>
 #include <trajectory_msgs/JointTrajectory.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <tf/transform_broadcaster.h>

#include <cmath>

class My_joint_pub{

  public:
    My_joint_pub();
    std::string controller_type;

  private:
    void callback(const geometry_msgs::PoseStamped::ConstPtr& data);
    void find_IK(const KDL::Frame &end_effector_pose);
    double eps,timeout;
    std::string chain_start, chain_end, urdf_param;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
     };

My_joint_pub::My_joint_pub(){\
  
  // load controller name
  ros::NodeHandle pnh("~");
  pnh.getParam("ctrl_topic", controller_type);

  nh.param("urdf_param", urdf_param, std::string("/robot_description"));//load urdf parametar from ROS
  chain_start = "base_link";//set chain start link link name corresponds urdf file
  chain_end = "ee_link";//set chain end link
  timeout = 0.005;//solvet time out 0.005=200Hz
  eps = 1e-4;//tolerance
  pub = nh.advertise<trajectory_msgs::JointTrajectory>(controller_type, 1);//set joint position publisher
  //pub = nh.advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_joint_traj_controller/command", 10);// real robot controll topic
  //pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);//sim robot controll topic
  sub = nh.subscribe("IK_target_pose", 10, &My_joint_pub::callback,this);// set subscriber listen end effector frame
}

void My_joint_pub::find_IK(const KDL::Frame &end_effector_pose){

  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);//check chain
  if (!valid){
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll, ul); //extract joint limit from urdf model
  if (!valid){
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }


  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  /*chechk model*/
  // ROS_INFO("Using %d joints!", chain.getNrOfJoints());
  // for(unsigned int i=0; i<chain.getNrOfJoints(); i++)
	// {
	// 	ROS_INFO("joint_name[%d]: %s", i, chain.getSegment(i).getJoint().getName().c_str());
	// 	ROS_INFO_STREAM("lower_joint_limits:"<<ll.data(i,0));
  //   ROS_INFO_STREAM("upper_joint_limits:"<<ul.data(i,0));
  // }


  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());
  for (uint j = 0; j < nominal.data.size(); j++){
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

  //create  ik result joint array
  KDL::JntArray result(ll.data.size());

  int rc=0;
  rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);//rc>0, IK solusion Found
  ROS_INFO_STREAM("trac ik result="<<rc);
  for (uint i=0;i<result.data.size();i++){
  ROS_INFO_STREAM("IK_result.data("<<i<<",0)="<<result.data(i,0));
    }

  if(rc>0){
    //set publish message
    trajectory_msgs::JointTrajectory tr0;
    tr0.header.frame_id ="base_link";
    tr0.joint_names.resize(6);
    tr0.points.resize(1);
    tr0.points[0].positions.resize(6);
    tr0.header.stamp = ros::Time::now();
    tr0.points[0].time_from_start = ros::Duration(0.02);
    for(unsigned int i=0; i<chain.getNrOfJoints(); i++){
        tr0.joint_names[i] = chain.getSegment(i).getJoint().getName().c_str();
        tr0.points[0].positions[i]=result(i);
      }
     pub.publish(tr0);
    }
  }


void My_joint_pub::callback(const geometry_msgs::PoseStamped::ConstPtr& data) {
    // ROS_INFO("Callbacking...");
    KDL::Vector end_pos(data->pose.position.x,data->pose.position.y,data->pose.position.z);
    KDL::Rotation end_rot=KDL::Rotation::Quaternion(data->pose.orientation.x,data->pose.orientation.y,data->pose.orientation.z,data->pose.orientation.w);
    KDL::Frame end_effector_pose(end_rot,end_pos);
    find_IK(end_effector_pose);
}



int main( int argc, char** argv )
{

  ros::init(argc, argv, "talker");

  My_joint_pub my1;

  ros::Rate loop_rate(200);
  while (ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
      }
}
