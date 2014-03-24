#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "goto_datamatrix");
	ros::NodeHandle node;
	ros::Rate rate(1.0); //for running the node in a loop at 1 hz
	
	ros::Publisher virtual_ob_pub = node.advertise<geometry_msgs::Pose>("/goal_end_effector_pose", 1);
	static tf::TransformBroadcaster tf_broadcaster;
	tf::StampedTransform flat_dm, virtual_ob, listened_transform, listened_virtual_ob;
	tf::TransformListener listener;
	
	double z_offset = 0.1;
	bool datamatrix_is_present = true;
	
	geometry_msgs::Pose virtual_ob_pose;
	
	while (node.ok())
	{
		datamatrix_is_present = true;
		try
		{
			listener.waitForTransform("/base_footprint", "/datamatrix_frame", ros::Time::now(), ros::Duration(0.8));
			listener.lookupTransform( "/base_footprint", "/datamatrix_frame", ros::Time(0), listened_transform);
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
			ROS_INFO("Waiting for /datamatrix_frame transform to appear...");
			datamatrix_is_present = false;
		}
		
		if(datamatrix_is_present)
		{
			//creating flat_datamatrix transform from base_footprint but shifted to datamatrix_frame position
			try
			{
				flat_dm.setOrigin( listened_transform.getOrigin() ); //shifting the origin
				flat_dm.setRotation( tf::Quaternion(0, 0, 0) ); //preserving the orientation of the parent (base_link)
				tf_broadcaster.sendTransform(tf::StampedTransform(flat_dm, ros::Time::now(), "base_footprint", "flat_datamatrix"));
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
			}
		
			//creating virtual_object frame = flat_datamatrix + z offset
			try
			{
				virtual_ob.setOrigin( tf::Vector3(0.0, 0.0, z_offset) ); //shifting the origin
				virtual_ob.setRotation( tf::Quaternion(0, 0, 0) ); //preserving the orientation of the parent (base_link)
				tf_broadcaster.sendTransform(tf::StampedTransform(virtual_ob, ros::Time::now(), "flat_datamatrix", "virtual_object"));
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
			}
		}
		
		rate.sleep();
	}
	return 0;
};
