#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "goto_datamatrix");
	ros::NodeHandle node("~");
	ros::Rate rate(1.0); //for running the node in a loop at 1 hz
	ros::Publisher datamatrix_virtual_object_pose_pub = node.advertise<geometry_msgs::Pose>("/goal_end_effector_pose", 10);
	static tf::TransformBroadcaster tf_broadcaster;
	tf::TransformListener listener;
	geometry_msgs::Pose datamatrix_virtual_object_coordinates;
	double z_offset = 0.1;
	
	tf::StampedTransform datamatrix_virtual_object, datamatrix_2_basefootprint_transform;
	
	while (node.ok())
	{
		try
		{
			ros::Time now = ros::Time::now();
			listener.waitForTransform("/datamatrix_frame", "/base_footprint", now, ros::Duration(0.8));
			listener.lookupTransform("/datamatrix_frame", "/base_footprint", ros::Time(0), datamatrix_2_basefootprint_transform);
			
			//transform_available = true;
			ROS_INFO("Transformation found! publishing datamatrix_virtual_object_coordinates now !");
			
			double 	datamatrix_x = datamatrix_2_basefootprint_transform.getOrigin().x(),
					datamatrix_y = datamatrix_2_basefootprint_transform.getOrigin().y(),
					datamatrix_z = datamatrix_2_basefootprint_transform.getOrigin().z() + z_offset;
			
			//broadcasting datamatrix_virtual_object frame
			datamatrix_virtual_object.setOrigin( tf::Vector3(datamatrix_x, datamatrix_y, datamatrix_z) ); //shifting the origin
			datamatrix_virtual_object.setRotation( tf::Quaternion(0, 0, 0) );
			tf_broadcaster.sendTransform(tf::StampedTransform(	datamatrix_virtual_object, 
																ros::Time::now(), 
																"base_footprint", 
																"datamatrix_virtual_object"));
			
			//packing twist message to publish on /goal_end_effector_pose
			datamatrix_virtual_object_coordinates.position.x = datamatrix_x;
			datamatrix_virtual_object_coordinates.position.y = datamatrix_y;
			datamatrix_virtual_object_coordinates.position.z = datamatrix_z;
			
			datamatrix_virtual_object_coordinates.orientation.x = datamatrix_2_basefootprint_transform.getRotation().x();
			datamatrix_virtual_object_coordinates.orientation.y = datamatrix_2_basefootprint_transform.getRotation().y();
			datamatrix_virtual_object_coordinates.orientation.z = datamatrix_2_basefootprint_transform.getRotation().z();
			datamatrix_virtual_object_coordinates.orientation.w = datamatrix_2_basefootprint_transform.getRotation().w();
			
			datamatrix_virtual_object_pose_pub.publish(datamatrix_virtual_object_coordinates);
			//return 0;
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
			ROS_INFO("Waiting for datamatrix transform to appear...");
		}
		
		rate.sleep();
	}
	
	ROS_INFO("Bye bye !");
	return 0;
};
