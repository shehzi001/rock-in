/*
 * main.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "rec_msg_and_serv/rec.h"

class RecDemo
{
  private:
    int kinect_trials_;
    int service_calls_;
    ros::NodeHandle n_;
    std::string camera_topic_;
    bool KINECT_OK_;

    void
    checkCloudArrive (const sensor_msgs::Image::ConstPtr& msg)
    {
      KINECT_OK_ = true;
    }

    void
    checkKinect ()
    {
      ros::Subscriber sub_pc = n_.subscribe (camera_topic_, 1, &RecDemo::checkCloudArrive, this);
      ros::Rate loop_rate (1);
      kinect_trials_ = 0;
      while (!KINECT_OK_ && ros::ok ())
      {
        std::cout << "Checking kinect status..." << std::endl;
        ros::spinOnce ();
        loop_rate.sleep ();
        kinect_trials_++;
        if(kinect_trials_ >= 5)
        {
          std::cout << "Kinect is not working..." << std::endl;
          return;
        }
      }

      KINECT_OK_ = true;
      std::cout << "Kinect is up and running" << std::endl;
    }

    void
    callService (const sensor_msgs::Image::ConstPtr& msg)
    {
      if( (service_calls_ % (30 * 5)) == 0)  // classify only every n-th incoming frame, e.g. (at 30 Hz) every 5 seconds
      {
        std::cout << "going to call service..." << std::endl;
        std::vector<std::string> models;
        ros::ServiceClient client = n_.serviceClient<rec_msg_and_serv::rec>("rec");
        rec_msg_and_serv::rec srv;
        srv.request.image = *msg;
        if (client.call(srv))
        {
          std::cout << "Found models:" << static_cast<int>(srv.response.models_found.size()) << std::endl;
          for(size_t i=0; i < srv.response.models_found.size(); i++)
          {
            std::cout << "   => " << srv.response.models_found[i] << "(" << srv.response.confidences_found[i] << ") ";
            std::cout << " [" << srv.response.poses_found[i].position.x <<
                         ", " << srv.response.poses_found[i].position.y <<
                         ", " << srv.response.poses_found[i].position.z << "]\n";
          }
        }
        else
        {
          std::cout << "service call failed" << std::endl;
        }
      }
      service_calls_++;
    }

  public:
    RecDemo()
    {
      KINECT_OK_ = false;
      camera_topic_ = "/camera/rgb/image_color";
      kinect_trials_ = 5;
    }

    bool initialize(int argc, char ** argv)
    {
      checkKinect();
      return KINECT_OK_;
    }

    void run()
    {
      ros::Subscriber sub_pc = n_.subscribe (camera_topic_, 1, &RecDemo::callService, this);
      ros::spin();
    }
};

int
main (int argc, char ** argv)
{
  ros::init (argc, argv, "rec_demo");

  RecDemo r;
  r.initialize (argc, argv);
  r.run();
  return 0;
}
