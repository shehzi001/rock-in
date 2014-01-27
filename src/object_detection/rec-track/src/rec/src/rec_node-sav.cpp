/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <QMainWindow>
#include "rec_msg_and_serv/rec.h"
#include "v4r/Tracker/Tracker.h"
#include "gltracker.h"
#include "sensor.h"
#include "recognizer.h"

class RecNode : public QMainWindow
{
  Q_OBJECT

public:
  explicit RecNode(QWidget *parent = 0);
  ~RecNode();

  private:
    ros::ServiceServer rec_service_;
    ros::NodeHandle n_;
    std::vector<std::string> models_;

    Ui::MainWindow *m_ui;
    GLTracker *m_gltracker;
    GLGraphicsView *m_glview;
    Sensor *m_sensor;
    Recognizer *m_recognizer;
    TomGine::tgModel m_model;

    bool recognize(rec_msg_and_serv::rec::Request & req,
                               rec_msg_and_serv::rec::Response & response)
    {
      std::cout << "start recognition ..\n";

      std::cout << ".. done recognition.\n";

      return true;
    }

    bool check_shader_path();
    void loadModel(const std::string &modelname);

  public:
    RecNode()
    : m_ui(new Ui::MainWindow)
    {
      m_ui->setupUi(this);

      check_shader_path();

      int cam_id;
      Tracking::Tracker::Parameter params;

      m_gltracker = new GLTracker(params, this);
      m_glview = new GLGraphicsView(m_ui->centralWidget);
      m_glview->setGeometry(QRect(10, 0, 640, 480));
      m_glview->setViewport(m_gltracker);

      m_sensor = new Sensor(params.camPar.width, params.camPar.height, cam_id);

      m_recognizer = new Recognizer(params);

      qRegisterMetaType<cv::Mat>("cv::Mat");
      connect(m_sensor, SIGNAL(new_image(cv::Mat)),
              m_gltracker, SLOT(new_image(cv::Mat)));
      connect(m_glview, SIGNAL(mouse_moved(QMouseEvent*)),
              m_gltracker, SLOT(mouse_moved(QMouseEvent*)));
      connect(m_glview, SIGNAL(mouse_pressed(QMouseEvent*)),
              m_gltracker, SLOT(mouse_pressed(QMouseEvent*)));
      connect(m_glview, SIGNAL(key_pressed(QKeyEvent*)),
              m_gltracker, SLOT(key_pressed(QKeyEvent*)));
      connect(m_gltracker, SIGNAL(send_track_time(float)),
              this, SLOT(track_time(float)));
      connect(m_gltracker, SIGNAL(send_tsd(Tracking::TrackingState,GLTracker::TSDSignals)),
              this, SLOT(track_state(Tracking::TrackingState,GLTracker::TSDSignals)));
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
  os << "OpenGL shader not found in '" << sp << "'" << endl;
  os << "You can set the shader path by modifying the definition for ";
  os << "TOMGINE_IP_SHADER in TomGineIP/CMakeLists.txt.";

  if(sp.empty())
    QMessageBox::critical(this, tr("Error"), tr(os.str().c_str()));

  std::string filename = sp + "ipGauss.frag";
  std::ifstream ifs(filename.c_str(), std::ios::in);
  if (!ifs)
    std::cerr << "ERROR: wrong shader path\n";

  return true;
}

void RecNode::loadModel(const std::string &filename)
{
  if(!m_gltracker->add_tracker_model(filename))
    std::cerr << "ERROR: failed to load model '" << filename << "'\n";
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
