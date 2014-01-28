#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "base_test");
	ros::NodeHandle n;
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Publisher arm_pub = n.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 1);
  
	geometry_msgs::Twist vel;
	brics_actuator::JointPositions pos;
	brics_actuator::JointValue joints;
	 
	//to give some time to robot
	ros::Duration(0.5).sleep();
	
	//go forward 5 secs
	vel.linear.x = 0.1; 
    vel_pub.publish(vel);
    ROS_INFO("drive forward");
    ros::Duration(5.0).sleep();
    

    //turn the base left, 4 secs, 0.3 rad
    vel.linear.x = 0;
    vel.angular.z = 0.5;
    vel_pub.publish(vel);
    ROS_INFO("turn left");
    ros::Duration(5.0).sleep();
    
    
    //move the manipulator to an arbitrary safe joint position
    vel.angular.z = 0;
    vel_pub.publish(vel);
    
    while(ros::ok()){
	
    //joint 1
    joints.joint_uri = "arm_joint_1";
    joints.unit = "rad";
    joints.value = static_cast<double>(2.9381);
    pos.positions.push_back(joints);
    arm_pub.publish(pos);
    
    //joint 2
    joints.joint_uri = "arm_joint_2";
    joints.unit = "rad";
    joints.value = static_cast<double>(1.04351);
    pos.positions.push_back(joints);
    arm_pub.publish(pos);
    
    //joint 3
    joints.joint_uri = "arm_joint_3";
    joints.unit = "rad";
    joints.value = static_cast<double>(-2.44154);
    pos.positions.push_back(joints);
    arm_pub.publish(pos);
    
    //joint 4
    joints.joint_uri = "arm_joint_4";
    joints.unit = "rad";
    joints.value = static_cast<double>(1.83102);
    pos.positions.push_back(joints);
    arm_pub.publish(pos);
    
    //joint 5
    joints.joint_uri = "arm_joint_5";
    joints.unit = "rad";
    joints.value = static_cast<double>(2.96447);
    pos.positions.push_back(joints);
    arm_pub.publish(pos);   
    ROS_INFO("move the manipulator");
 
    
    //shift the base right, 0.2 m/s, 3sec
    vel.linear.y = -0.1;
    vel_pub.publish(vel);
    ROS_INFO("shift right");
    ros::Duration(5.0).sleep();
    
    //move the manipulator back the initial position
    vel.linear.y = 0;
    vel_pub.publish(vel);  
    
    //joint 1
    joints.joint_uri = "arm_joint_1";
    joints.unit = "rad";
    joints.value = static_cast<double>(0.0100692);
    pos.positions.push_back(joints);
    arm_pub.publish(pos);
    
    //joint 2
    joints.joint_uri = "arm_joint_2";
    joints.unit = "rad";
    joints.value = static_cast<double>(0.0100693);
    pos.positions.push_back(joints);
    arm_pub.publish(pos);
    
    //joint 3
    joints.joint_uri = "arm_joint_3";
    joints.unit = "rad";
    joints.value = static_cast<double>(-5.02654);
    pos.positions.push_back(joints);
    arm_pub.publish(pos);
    
    //joint 4
    joints.joint_uri = "arm_joint_4";
    joints.unit = "rad";
    joints.value = static_cast<double>( 0.0221239);
    pos.positions.push_back(joints);
    arm_pub.publish(pos);
    
    //joint 5
    joints.joint_uri = "arm_joint_5";
    joints.unit = "rad";
    joints.value = static_cast<double>(0.110620);
    pos.positions.push_back(joints);
    arm_pub.publish(pos);
    ROS_INFO("move manipulator back");
	
	
	ros::Duration(0.5).sleep();
    ros::spinOnce();
  }
  return 0;
}
