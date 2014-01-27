/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include "rec_msg_and_serv/rec.h"
#include "v4r/Tracker/Tracker.h"
#include "recognizer.h"

class RecNode
{
  private:
    ros::ServiceServer rec_service_;
    ros::NodeHandle n_;
    std::vector<std::string> models_;

    Recognizer *m_recognizer;
    TomGine::tgModel m_model;

    bool recognize(rec_msg_and_serv::rec::Request & req,
                   rec_msg_and_serv::rec::Response & response)
    {
      std::cout << "start recognition ..\n";

      for(size_t i = 0; i < models_.size(); i++)
      {
        std::cout << "checking model '" << models_[i] << "'\n";
        loadModel(models_[i]);
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(req.image, "bgr8");;
        cv::imshow("to recognize", cv_image->image);
        cv::waitKey(10);
        TomGine::tgPose pose;
        float confidence = 0.;
        recognizeModel(cv_image->image, pose, confidence);
        if(confidence > 0.)
        {
          std::cout << "recognized '" << models_[i] << "' with confidence " << confidence << std::endl;
          std::string name = models_[i].substr(models_[i].find_last_of('/'), models_[i].npos);
          std_msgs::String ss;
          ss.data = name;
          geometry_msgs::Pose gPose;
          gPose.position.x = pose.t.x;
          gPose.position.y = pose.t.y;
          gPose.position.z = pose.t.z;
          gPose.orientation.x = pose.q.x;
          gPose.orientation.y = pose.q.y;
          gPose.orientation.z = pose.q.z;
          gPose.orientation.w = pose.q.w;
          response.models_found.push_back(ss);
          response.confidences_found.push_back(confidence);
          response.poses_found.push_back(gPose);
        }
      }

      std::cout << ".. done recognition.\n";

      return true;
    }

    bool check_shader_path();
    void loadModel(const std::string &modelname);
    void recognizeModel(const cv::Mat& img, TomGine::tgPose& pose, float &confidence);

  public:
    RecNode()
    {
      check_shader_path();
      Tracking::Tracker::Parameter params;

      m_recognizer = new Recognizer(params);
    }

    ~RecNode()
    {
      delete m_recognizer;
    }

    void initialize(int argc, char ** argv)
    {
      for(int i = 1; i < argc; i++)
      {
        std::cout << "model: " << argv[i] << "\n";
        models_.push_back(argv[i]);
      }

      if(models_.empty())
      {
        std::cerr << "need at leat one model specified, ABORTING\n";
        return;
      }

      rec_service_ = n_.advertiseService("rec", &RecNode::recognize, this);
    }

    void run()
    {
      ros::spin();
    }
};

bool RecNode::check_shader_path()
{
  std::string sp = TomGine::tgImageProcessor::getShaderPath();

  std::ostringstream os;
  os << "OpenGL shader not found in '" << sp << "'" << std::endl;
  os << "You can set the shader path by modifying the definition for ";
  os << "TOMGINE_IP_SHADER in TomGineIP/CMakeLists.txt.";

  if(sp.empty())
    std::cerr << os.str() << std::endl;

  std::string filename = sp + "ipGauss.frag";
  std::ifstream ifs(filename.c_str(), std::ios::in);
  if (!ifs)
    std::cerr << os.str() << std::endl;

  return true;
}

void RecNode::loadModel(const std::string &filename)
{
  m_recognizer->load(filename);
}

void RecNode::recognizeModel(const cv::Mat& img, TomGine::tgPose& pose, float &confidence)
{
  m_recognizer->recognize(img, pose, confidence);
}

int
main (int argc, char ** argv)
{
  ros::init (argc, argv, "rec");

  RecNode r;
  r.initialize (argc, argv);
  r.run();

  return 0;
}
