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
#include "gltracker.h"

using namespace TomGine;
using namespace Tracking;
using namespace std;

GLTracker::GLTracker(Tracker::Parameter& params, QWidget* _parent)
  : QGLWidget(_parent),
    m_timer(this),
    m_tracker(NULL),
    m_model_id(-1)
{

  setAttribute(Qt::WA_NoSystemBackground,true);
  setFocusPolicy(Qt::StrongFocus);
  setAcceptDrops(true);
  setCursor(Qt::PointingHandCursor);

  this->setGeometry(QRect(0,0,params.camPar.width,params.camPar.height));

  m_params = params;

  connect(&m_timer, SIGNAL(timeout()), this, SLOT(draw()));
  m_timer.start(50);
  m_elapsed.start();
}

GLTracker::~GLTracker()
{
  if(m_tracker)
    delete m_tracker;
}

void GLTracker::set_parameters(Tracking::Tracker::Parameter params)
{
  m_tracker->setParameter(params);
}

void GLTracker::new_image(cv::Mat img)
{
  m_image = img; // TODO: don't know if this is save
  //  cv::Mat mat(img);
  //  mat.copyTo(m_image);
}

void GLTracker::draw()
{
  updateGL();
}

void GLTracker::mouse_moved(QMouseEvent *event)
{
  this->mouseMoveEvent(event);
}

void GLTracker::mouse_pressed(QMouseEvent *event)
{
  this->mousePressEvent(event);
}

void GLTracker::key_pressed(QKeyEvent *event)
{
  this->keyPressEvent(event);
}

bool GLTracker::add_tracker_model(QString& filename)
{
  TomGine::tgPose p;
  m_model_id = m_tracker->addModelFromFile(filename.toStdString().c_str(), p, "model");
  m_tracker->setLockFlag(true);

  cout << endl; // flush printf outputs
  updateGL();

  if(m_model_id==-1)
    return false;
  return true;

}

const TomGine::tgModel& GLTracker::get_model()
{
  return m_tracker->getModelEntry(m_model_id)->model;
}

TomGine::tgPose GLTracker::get_pose()
{
  TomGine::tgPose pose;
  m_tracker->getModelPoseCamCoords(m_model_id, pose);
  return pose;
}

void GLTracker::set_pose(const TomGine::tgPose &pose)
{
  m_tracker->setModelInitialPose(m_model_id, pose);
  m_tracker->reset();
}

void GLTracker::start_tracking()
{
  m_tracker->setLockFlag(false);
  m_timer.stop();
  m_timer.start(1);
}

void GLTracker::stop_tracking()
{
  m_tracker->setLockFlag(true);
}

void GLTracker::reset_tracking()
{
  m_tracker->reset();
}

void GLTracker::switch_display()
{
  m_tracker->setModelModeFlag(m_tracker->getModelModeFlag() + 1);
}

bool GLTracker::set_tsd(bool on)
{
  bool full_textured = m_tracker->getModelEntry(m_model_id)->model.getFullTextured();
  m_tracker->setTrackingStateDetection(on);

  if(on && !full_textured)
    return false;
  return true;
}

void GLTracker::clear_texture()
{
  m_tracker->getModelEntry(m_model_id)->model.releasePassList();
}

void GLTracker::grab_texture()
{
  m_tracker->textureFromImage();
}

bool GLTracker::model_loaded()
{
  if(m_model_id>=0)
    return true;

  return false;
}

void GLTracker::save_model(std::string filename)
{
  m_tracker->saveModel(m_model_id, filename.c_str());
}

void GLTracker::initializeGL()
{ 
  m_tracker = new TextureTracker();
  m_tracker->init(m_params);
  m_cam_origin = m_cam_perspective = m_tracker->getCamera();
  m_cam_ortho.Set(vec3(0, 0, 1), vec3(0, 0, 0), vec3(0, 1, 0), 45, width(), height(), 0.1f, 2.0f, tgCamera::GL_ORTHO);
}

void GLTracker::resizeGL(int w, int h)
{
  m_cam_perspective.SetViewport(w, h);
  m_cam_perspective.Activate();
  updateGL();
}

void GLTracker::addNewModel()
{
  //  if(!m_filename.isEmpty())
  //  {
  //    TomGine::tgPose p;
  //    m_model_id = m_tracker->addModelFromFile(m_filename.toStdString().c_str(), p, "model");
  //    m_filename.clear();
  //    m_tracker->setLockFlag(true);
  ////    m_tracker->getModelPose(m_model_id, p);
  ////    cout << "model pose: t: " << p.t.x << " " << p.t.y << " " << p.t.z << endl;
  ////    m_tracker->getModelEntry(m_model_id)->model.Print();
  ////    cout << endl;
  //  }
}

void GLTracker::drawCoordinates(float length)
{
  m_cam_perspective.Activate();
  glDisable(GL_LIGHTING);
  glLineWidth(2.0);
  glBegin(GL_LINES);
  glColor3ub(255,0,0); glVertex3f(0.0f,0.0f,0.0f); glVertex3f(length,0.0f,0.0f);
  glColor3ub(0,255,0); glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f,length,0.0f);
  glColor3ub(0,0,255); glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f,0.0f,length);
  glEnd();
}


void GLTracker::displayStates()
{
  TrackingState ts;
  m_tracker->getModelTrackingState(m_model_id, ts);
  ModelEntry* modelentry = m_tracker->getModelEntry(m_model_id);

  float font_size = 20.0f;
  if (ts == ST_OK)
    g_font->Print ("state: ok", font_size, 10, font_size, 0, 0, 0);
  else if (ts == ST_OCCLUDED)
    g_font->Print ("state: occluded", font_size, 10, font_size, 0, 0, 0);
  else if (ts == ST_LOST)
    g_font->Print ("state: lost", font_size, 10, font_size, 1, 0, 0);
  else if (ts == ST_LOCKED)
    g_font->Print ("state: locked", font_size, 10, font_size);

  std::ostringstream os;
  os << "quality: " << unsigned (100 * modelentry->c_normalized) << "%";
  g_font->Print (os.str ().c_str (), font_size, 10, 2 * font_size, 1.0f - modelentry->c_normalized, 0, 0);

  os.str ("");
  os << "static: " << unsigned (100 * modelentry->w_msp) << "%";
  g_font->Print (os.str ().c_str (), font_size, 10, 3 * font_size, 1.0f - modelentry->w_msp, 0, 0);

  os.str ("");
  os << "c_occ: " << unsigned (100 * std::min<float> (5.0f * modelentry->c_occ_comp, 1.0f));
  g_font->Print (os.str ().c_str (), font_size, 10, 4 * font_size,
                 std::min<float> (5.0f * modelentry->c_occ_comp, 1.0f), 0, 0);
}

void GLTracker::track()
{
  if(!m_image.empty() && m_model_id>=0)
  {
    m_elapsed.restart();
    m_tracker->image_processing(m_image.data);
    m_tracker->track(m_model_id);
    emit send_track_time(0.001 * m_elapsed.elapsed());

    m_tracker->drawImage();

    //    displayStates();
    TrackingState ts = ST_DISABLED;
    TSDSignals sig;
    if(!m_tracker->getLockFlag() && m_tracker->getTrackingStateDetection())
    {
      m_tracker->getModelTrackingState(m_model_id, ts);
      ModelEntry* modelentry = m_tracker->getModelEntry(m_model_id);
      sig.convergence = modelentry->w_msp;
      sig.quality = modelentry->c_normalized;
      sig.occlusion = std::min<float> (5.0f * modelentry->c_occ_comp, 1.0f);
      sig.loss = modelentry->n_eff / modelentry->distribution.size();
    }
    emit send_tsd(ts, sig);

  }else{
    drawImage();
  }

  if(model_loaded())
    m_tracker->drawResult(m_model_id);
}

void GLTracker::drawImage()
{
  if(!m_image.empty())
  {
    m_cam_ortho.Activate();

    TomGine::tgTexture2D tex;
    tex.Load(m_image.data, m_image.cols, m_image.rows, GL_RGB, GL_BGR, GL_UNSIGNED_BYTE);

    float w = width();
    float h = height();

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);

    glColor3f(1,1,1);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 1.0f);
    glVertex3f(0, 0, 0.0f);
    glTexCoord2f(1.0f, 1.0f);
    glVertex3f(w, 0, 0.0f);
    glTexCoord2f(1.0f, 0.0f);
    glVertex3f(w, h, 0.0f);
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f(0, h, 0.0f);
    glEnd();

    glEnable(GL_DEPTH_TEST);
  }
}


void GLTracker::paintGL()
{
  // enable GL context
  makeCurrent();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //  addNewModel();
  track();
  //  drawCoordinates();
}


void GLTracker::mousePressEvent(QMouseEvent* event)
{
  //  cout << "GLWidget::mousePressEvent" << endl;
  m_last_point_2d = event->pos();
  updateGL();
}

void GLTracker::mouseMoveEvent(QMouseEvent* event)
{
  // enable GL context
  makeCurrent();

  QPoint newPoint2D = event->pos();

  float dx = newPoint2D.x() - m_last_point_2d.x();
  float dy = newPoint2D.y() - m_last_point_2d.y();

  float far = m_cam_perspective.GetZFar();
  float near = m_cam_perspective.GetZNear();

  // move in z direction
  if ( (event->buttons() == Qt::MidButton) )
  {
    m_cam_perspective.TranslateF(0.001f * (far - near) * dx);
    m_cam_perspective.TranslateF(0.001f * (far - near) * dy);
  }  // move in x,y direction
  else if ( (event->buttons() == Qt::RightButton) )
  {
    m_cam_perspective.TranslateS(-0.0005f * (far - near) * dx);
    m_cam_perspective.TranslateU(0.0005f * (far - near) * dy);
  } // rotate
  else if (event->buttons() == Qt::LeftButton)
  {
    TomGine::vec3 cor(0, 0, 0);
    m_cam_perspective.Orbit(cor, m_cam_perspective.GetU(), -0.05f * dx);
    m_cam_perspective.Orbit(cor, m_cam_perspective.GetS(), -0.05f * dy);
  }

  m_cam_perspective.ApplyTransform();

  // remember this point
  m_last_point_2d = newPoint2D;

  // trigger redraw
  updateGL();
}

void GLTracker::wheelEvent(QWheelEvent* event)
{
  float d = -(float)event->delta() / 120.0 * 0.2;

  float far = m_cam_perspective.GetZFar();
  float near = m_cam_perspective.GetZNear();
  m_cam_perspective.TranslateF(0.1f * (far - near) * d);
  m_cam_perspective.TranslateF(0.1f * (far - near) * d);
  m_cam_perspective.ApplyTransform();

  updateGL();
  event->accept();
}

void GLTracker::keyPressEvent(QKeyEvent *event)
{
  if(event->key() == Qt::Key_Z)
  {
    m_cam_perspective = m_cam_origin;
    m_cam_perspective.Activate();
  }
  if(event->key() == Qt::Key_O)
  {
    m_cam_perspective.Print();
  }
  updateGL();
}

