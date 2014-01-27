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
#include <datamatrix_finder/Datamatrix.h>

// defining variables for input and output topics
std::string inputTopic;
std::string outputTopic;

// declaring subscriber and publisher
ros::Subscriber imageSub;
ros::Publisher imagePub;

datamatrix_finder::Datamatrix msg;

void dataCallback(const std_msgs::String::ConstPtr& msg);

void dataCallback(const std_msgs::String::ConstPtr& msg){
	msg.message = dataMatrixInfo.message;
	cout<<msg.message<<endl;
}



int main(int argc, char** argv) {

	ros::init(argc, argv, "agent_marker_localization_node");
	ros::NodeHandle nh("~");
	
	imageSub = nh.subscribe(dataMatrix_finder/datamatrix_pub, 1, dataCallback);
	//imagePub = nh.advertise<cv_bridge::CvImage>(outputTopic, 1);

	server.setCallback(f);
	
	ros::spin();
	return 0;

}
