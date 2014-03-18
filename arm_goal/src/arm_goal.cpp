#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "arm_goal");
	ros::NodeHandle node;
	ros::Rate rate(1.0); //for running the node in a loop at 1 hz
	
	ros::Publisher virtual_ob_pub = node.advertise<geometry_msgs::Pose>("/goal_end_effector_pose", 1);
	static tf::TransformBroadcaster tf_broadcaster;
	tf::StampedTransform flat_dm, virtual_ob, listened_transform, listened_virtual_ob;
	tf::TransformListener listener;
	
	double z_offset = 0.1;
	bool virtual_object_available = true;
	geometry_msgs::Pose virtual_ob_pose;
	
	while (node.ok())
	{
		try
		{
			listener.waitForTransform("/base_footprint", "/virtual_object", ros::Time::now(), ros::Duration(0.8));
			listener.lookupTransform( "/base_footprint", "/virtual_object", ros::Time(0), listened_virtual_ob);
			ROS_INFO("Virtual object is available !");
			virtual_object_available = true;
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
			ROS_INFO("Waiting for /virtual_object transform to appear...");
			virtual_object_available = false;
		}
		
		if(virtual_object_available == true)
		{
			/*
			//debug stuff
			double x = listened_virtual_ob.getOrigin().x();
			float x2 = listened_virtual_ob.getOrigin().x();
			
			ROS_INFO("&&&&&&&&&&&&&");
			ROS_INFO_STREAM(x);
			ROS_INFO_STREAM(x2);
			ROS_INFO_STREAM(listened_virtual_ob.getOrigin().x());
			ROS_INFO("&&&&&&&&&&&&&");
			*/
			
			//packing twist message to publish on /goal_end_effector_pose
			virtual_ob_pose.position.x = listened_virtual_ob.getOrigin().x();
			virtual_ob_pose.position.y = listened_virtual_ob.getOrigin().y();
			virtual_ob_pose.position.z = listened_virtual_ob.getOrigin().z();
			
			virtual_ob_pose.orientation.x = listened_virtual_ob.getRotation().x();
			virtual_ob_pose.orientation.y = listened_virtual_ob.getRotation().y();
			virtual_ob_pose.orientation.z = listened_virtual_ob.getRotation().z();
			virtual_ob_pose.orientation.w = listened_virtual_ob.getRotation().w();
			
			ROS_INFO("Publishing virtual object pose now !");
			virtual_ob_pub.publish(virtual_ob_pose);
			return 0; //kill node after publishing
		}
		
		rate.sleep();
	}
	return 0;
};
