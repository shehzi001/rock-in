/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2011, Thomas Mörwald
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author thomas.moerwald
 *
 */
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>

Q_DECLARE_METATYPE(cv::Mat)

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  m_ui(new Ui::MainWindow),
  m_params(new Params(this))
{
  m_ui->setupUi(this);

  check_shader_path();

  int cam_id;
  Tracking::Tracker::Parameter params;
  m_params->get_parameters(cam_id, params);

  m_gltracker = new GLTracker(params, this);
  m_glview = new GLGraphicsView(m_ui->centralWidget);
  m_ui->trackingView = m_glview;
  m_glview->setGeometry(QRect(10, 0, 640, 480));
  m_glview->setViewport(m_gltracker);

  m_sensor = new Sensor(params.camPar.width, params.camPar.height, cam_id);

  m_recognizer = new Recognizer(params);

  qRegisterMetaType<cv::Mat>("cv::Mat");
  connect(m_params, SIGNAL(send_sensor_parameters(double,double,int)),
          m_sensor, SLOT(set_parameter(double,double,int)));
  connect(m_params, SIGNAL(send_tracking_parameters(Tracking::Tracker::Parameter)),
          m_gltracker, SLOT(set_parameters(Tracking::Tracker::Parameter)));
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

  setWindowTitle(tr("Tracker"));
}

MainWindow::~MainWindow()
{
  if(m_params->parameters_changed())
  {
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Save parameters",
                                  "Tracking parameters have changed. Do you want to save them?",
                                  QMessageBox::Yes|QMessageBox::No);

    if(reply==QMessageBox::Yes)
      m_params->save_parameters();
  }

  delete m_ui;
  delete m_gltracker;
  delete m_glview;
  delete m_sensor;
  delete m_recognizer;
  delete m_params;
}


void MainWindow::on_ButtonStart_pressed()
{
  if(m_gltracker->model_loaded())
    m_gltracker->start_tracking();
  else
    QMessageBox::critical(this, tr("Error"), tr("No model loaded. To load a model click 'File->Load Model'"));
}

void MainWindow::on_ButtonStop_pressed()
{
  if(!m_gltracker->model_loaded())
    return;

  m_gltracker->stop_tracking();
}

void MainWindow::on_ButtonReset_pressed()
{
  if(!m_gltracker->model_loaded())
    return;

  m_gltracker->reset_tracking();
}

void MainWindow::on_actionLoad_Model_triggered()
{
  int cam_id;
  Tracking::Tracker::Parameter params;
  m_params->get_parameters(cam_id, params);

  QString model_path(params.modelPath.c_str());
  QString filename = QFileDialog::getOpenFileName(this,tr("Open file..."),model_path,tr("*.ply"));

  if(filename.size()==0)
    return;

  cout << "Load model: " << filename.toStdString() << endl;
  if(!m_gltracker->add_tracker_model(filename))
    QMessageBox::critical(this, tr("Error"), tr("Failed to load ply model."));
}

void MainWindow::on_actionSave_Model_triggered()
{
  if(!m_gltracker->model_loaded())
    return;

  // get filename from QFileDialog
  int cam_id;
  Tracking::Tracker::Parameter params;
  m_params->get_parameters(cam_id, params);
  QString model_path(params.modelPath.c_str());
  QString filename = QFileDialog::getSaveFileName(this, tr("Save PLY file..."), model_path, tr("*.ply"));

  if(filename.size()==0)
    return;

  string fn = filename.toStdString();
  make_extension(fn, ".ply");

  // save
  cout << "Save Model: " << fn << endl;
  m_gltracker->save_model(fn);
}

void MainWindow::on_actionLoad_SIFT_Model_triggered()
{
  int cam_id;
  Tracking::Tracker::Parameter params;
  m_params->get_parameters(cam_id, params);

  QString model_path(params.modelPath.c_str());
  QString filename = QFileDialog::getOpenFileName(this,tr("Open file..."),model_path,tr("*.sift"));

  if(filename.size()==0)
    return;

  cout << "Load SIFT model: " << filename.toStdString() << endl;
  m_recognizer->load(filename.toStdString());
}

void MainWindow::on_actionSave_SIFT_Model_triggered()
{
  int cam_id;
  Tracking::Tracker::Parameter params;
  m_params->get_parameters(cam_id, params);

  QString model_path(params.modelPath.c_str());
  QString filename = QFileDialog::getSaveFileName(this, tr("Save SIFT file..."), model_path, tr("*.sift"));

  if(filename.size()==0)
    return;

  string fn = filename.toStdString();
  make_extension(fn, ".sift");

  // save
  cout << "Save SIFT Model: " << fn << endl;
  m_recognizer->save(fn);
}

void MainWindow::on_actionPreferences_triggered()
{
  m_params->show();
}

void MainWindow::on_actionExit_triggered()
{
  QApplication::exit();
}

void MainWindow::on_ButtonStartRGB_pressed()
{
  m_sensor->start();
}

void MainWindow::on_ButtonStopRGB_pressed()
{
  m_sensor->stop();
}

void MainWindow::on_ButtonDisplay_pressed()
{
  if(!m_gltracker->model_loaded())
    return;

  m_gltracker->switch_display();
}

void MainWindow::on_ButtonLearn_pressed()
{
  if(!m_gltracker->model_loaded())
    return;

  cv::Mat img;
  const TomGine::tgModel& model = m_gltracker->get_model();
  const TomGine::tgPose& pose = m_gltracker->get_pose();

  m_sensor->get_image(img);
  m_recognizer->learn(img, model, pose);
}

void MainWindow::on_ButtonRecognize_pressed()
{
  if(!m_gltracker->model_loaded())
    return;

  cv::Mat img;
  TomGine::tgPose pose;
  float confidence(0.0f);

  m_sensor->get_image(img);
  m_recognizer->recognize(img, pose, confidence);

  if(confidence > 0.0)
    m_gltracker->set_pose(pose);
}

void MainWindow::on_ButtonGrabTexture_pressed()
{
  if(!m_gltracker->model_loaded())
    return;

  m_gltracker->grab_texture();
}

void MainWindow::on_ButtonClearTexture_pressed()
{
  if(!m_gltracker->model_loaded())
    return;

  m_gltracker->clear_texture();
}

void MainWindow::track_time(float track_time)
{
  static Tracking::FloatFilter lpTime(0.95);
  lpTime = 1.0 / track_time;
  m_ui->labelFPSVal->setText(QString::number((unsigned)lpTime));
}

void MainWindow::track_state(Tracking::TrackingState ts, GLTracker::TSDSignals sig)
{
  m_ui->labelConvergenceVal->setText(QString::number((unsigned)(100.0 * sig.convergence)));
  m_ui->labelQualityVal->setText(QString::number((unsigned)(100.0 * sig.quality)));
  m_ui->labelOcclusionVal->setText(QString::number((unsigned)(100.0 * sig.occlusion)));
  m_ui->labelLossVal->setText(QString::number((unsigned)(100.0 * sig.loss)));

  if(ts==Tracking::ST_DISABLED)
    m_ui->labelTSDVal->setText(QString("disabled"));
  else if(ts==Tracking::ST_OK)
    m_ui->labelTSDVal->setText(QString("ok"));
  else if(ts==Tracking::ST_OCCLUDED)
    m_ui->labelTSDVal->setText(QString("occluded"));
  else if(ts==Tracking::ST_LOST)
    m_ui->labelTSDVal->setText(QString("lost"));
}

void MainWindow::on_checkTSD_toggled(bool checked)
{
  if(!m_gltracker->set_tsd(checked))
    QMessageBox::warning(this, tr("Warning"), tr("State detection only works correctly when model is fully textured"));
}

bool MainWindow::check_shader_path()
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
    QMessageBox::critical(this, tr("Error"), tr(os.str().c_str()));

  return true;
}

void MainWindow::make_extension(string& filename, string ext)
{
  if(ext.find_first_of(".")!=0)
    ext.insert(0, ".");

  std::string::size_type idx_dot = filename.rfind('.');
  std::string::size_type idx_sl = filename.rfind('/');
  std::string::size_type idx_bsl = filename.rfind('\\');
  if((idx_dot == std::string::npos) ||
     ((idx_dot < idx_sl) && (idx_sl != std::string::npos)) ||
     ((idx_dot < idx_bsl) && (idx_bsl != std::string::npos)))
    filename.append(ext);
}




