/*
 * File : agent_marker_localization_node.cpp 
 * Authors : Bipin Kumar
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>
#include <datamatrix_finder/Datamatrix.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// declaring subscriber and publisher
ros::Subscriber imageSub;
ros::Publisher imagePub;
ros::Subscriber sub;

using namespace std; 
/*
void chatterCallback(const datamatrix_finder::Datamatrix::ConstPtr& msg)
{
	
	ros::Time now = ros::Time::now();
	tf::StampedTransform transform;
	tf::TransformListener listener;

	//ROS_INFO("Datamatrix_to_Camera : [%f,%f,%f]", msg->translation[0], msg->translation[1], msg->translation[2]);
	
	//imageSub = nh.subscribe("/datamatrix", 1, dataCallback);
	listener.waitForTransform("/base_footprint","/datamatrix_frame", ros::Time(0), ros::Duration(2.0));
	
	 try
	{
      listener.lookupTransform("/base_footprint","/datamatrix_frame", ros::Time(0), transform);
      ROS_INFO("base_to_datamatrix: [%f,%f,%f]",transform.getOrigin().x(),transform.getOrigin().y(), transform.getOrigin().z());
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    //double x = msg->translation[0] + transform.getOrigin().x();
    //double y = msg->translation[1] + transform.getOrigin().y();
    //double z = msg->translation[2] + transform.getOrigin().z();
    
   // ROS_INFO("Datamatrix_to_Base: [%f, %f, %f]", x, y, z);
}
*/
int main(int argc, char** argv) {

	ros::init(argc, argv, "agent_marker_localization_node");
	ros::NodeHandle nh;
	//sub = nh.subscribe("datamatrix", 1000, chatterCallback);
	ROS_INFO("agent_marker_localization_node");
	
	
	tf::StampedTransform transform;
	tf::TransformListener listener;
	ros::Rate rate(10.0);
	
	while(nh.ok())
	{
	
	//listener.waitForTransform("/base_footprint","/datamatrix_frame", ros::Time(0), ros::Duration(2.0));
	/*
	 try{
      listener.lookupTransform("/base_footprint","/datamatrix_frame", ros::Time(0), transform);
      ROS_INFO("Camera_to_Base: [%f,%f,%f]",transform.getOrigin().x(),transform.getOrigin().y(), transform.getOrigin().z());
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
	*/
	
	ros::Time now = ros::Time::now();
	bool transform_flag = listener.waitForTransform("/base_footprint","/datamatrix_frame", now, ros::Duration(1.0));
		
			try 
			{   
				listener.lookupTransform("/base_footprint","/datamatrix_frame", now , transform);
				ROS_INFO("Camera_to_Base: [%f,%f,%f]",transform.getOrigin().x(),transform.getOrigin().y(), transform.getOrigin().z());
			} catch (tf::TransformException ex) {
				ROS_ERROR("%s",ex.what());
			}
		
	}
	 
	
	ros::spin();
	return 0;

}
