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
#ifndef _TRACKER_QT_GLTRACKER_H_
#define _TRACKER_QT_GLTRACKER_H_

#include <qgraphicswidget.h>
#include <QGLWidget>
#include <QMouseEvent>
#include <QTimer>
#include <QElapsedTimer>
#include <QThread>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include "v4r/TomGine/tgCamera.h"
#include "v4r/TomGine/tgTexture.h"

#include "v4r/Tracker/TextureTracker.h"

#include <QtGui/QGraphicsView>
class GLGraphicsView : public QGraphicsView
{
  Q_OBJECT

signals:
  void mouse_moved(QMouseEvent *event);
  void mouse_pressed(QMouseEvent *event);
  void key_pressed(QKeyEvent *event);

public:
  GLGraphicsView(QWidget* widget=0) : QGraphicsView(widget) { }

  void mouseMoveEvent(QMouseEvent *event)
  {
    emit mouse_moved(event);
  }

  void mousePressEvent(QMouseEvent *event)
  {
    emit mouse_pressed(event);
  }

  void keyPressEvent(QKeyEvent *event)
  {
    emit key_pressed(event);
  }
};

class GLTracker : public QGLWidget
{
  Q_OBJECT

public:
  //! Default constructor.
  GLTracker(Tracking::Tracker::Parameter &params, QWidget* _parent=0);

  //! Destructor.
  virtual ~GLTracker();

  struct TSDSignals
  {
    TSDSignals() : convergence(0), quality(0), occlusion(0), loss(0) {}
    float convergence;
    float quality;
    float occlusion;
    float loss;
  };

  bool add_tracker_model(QString& filename);
  const TomGine::tgModel& get_model();
  TomGine::tgPose get_pose();
  void set_pose(const TomGine::tgPose &pose);
  void start_tracking();
  void stop_tracking();
  void reset_tracking();
  void switch_display();
  bool set_tsd(bool on);
  void clear_texture();
  void grab_texture();
  bool model_loaded();
  void save_model(std::string filename);

signals:
  void send_track_time(float fps);
  void send_tsd(Tracking::TrackingState ts, GLTracker::TSDSignals sig);

public slots:
  void set_parameters(Tracking::Tracker::Parameter params);
  void new_image(cv::Mat img);
  void draw();

  void mouse_moved(QMouseEvent *event);
  void mouse_pressed(QMouseEvent *event);
  void key_pressed(QKeyEvent *event);

private:

  //! Initializes OpenGL states (triggered by Qt).
  void initializeGL();

  //! Adds model from file to tracker
  void addNewModel();

  //! Draws a coordinate frame at the origin (0,0,0).
  void drawCoordinates(float length=1.0);

  void displayStates();

  //! Tracks an object
  void track();

  //! Grabs an Image and draws it.
  void drawImage();

  //! Draws the scene (triggered by Qt).
  void paintGL();

  //! Handle resize events (triggered by Qt).
  void resizeGL(int w, int h);

  TomGine::tgCamera m_cam_origin;
  TomGine::tgCamera m_cam_perspective;
  TomGine::tgCamera m_cam_ortho;
  QPoint m_last_point_2d;
  QElapsedTimer m_elapsed;
  QTimer m_timer;
  cv::Mat m_image;
  Tracking::TextureTracker* m_tracker;
  Tracking::TextureTracker::Parameter m_params;
//  QString m_filename;
  int m_model_id;

protected:

  // Qt mouse events
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);
  virtual void wheelEvent(QWheelEvent* event);
  virtual void keyPressEvent(QKeyEvent *event);

};



#endif // GLWIDGET_H
