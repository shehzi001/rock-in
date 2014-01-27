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
#ifndef _TRACKER_QT_MAINWINDOW_H_
#define _TRACKER_QT_MAINWINDOW_H_

#include <QMainWindow>
#include "gltracker.h"
#include "sensor.h"
#include "params.h"
#include "recognizer.h"
#include "v4r/Tracker/Tracker.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT
  
public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

signals:

private slots:
  void on_ButtonStart_pressed();
  void on_ButtonStop_pressed();
  void on_ButtonReset_pressed();

  void on_actionLoad_Model_triggered();
  void on_actionSave_Model_triggered();
  void on_actionSave_SIFT_Model_triggered();
  void on_actionPreferences_triggered();
  void on_actionExit_triggered();

  void on_ButtonStartRGB_pressed();
  void on_ButtonStopRGB_pressed();
  void on_ButtonDisplay_pressed();
  void on_ButtonLearn_pressed();
  void on_ButtonRecognize_pressed();
  void on_ButtonGrabTexture_pressed();
  void on_ButtonClearTexture_pressed();

  void track_time(float track_time);
  void track_state(Tracking::TrackingState ts, GLTracker::TSDSignals sig);

  void on_checkTSD_toggled(bool checked);

  void on_actionLoad_SIFT_Model_triggered();

private:
  bool check_shader_path();
  void make_extension(std::string& filename, std::string ext);

  Ui::MainWindow *m_ui;
  GLTracker *m_gltracker;
  GLGraphicsView *m_glview;
  Sensor *m_sensor;
  Params *m_params;
  Recognizer *m_recognizer;
  TomGine::tgModel m_model;

};

#endif // MAINWINDOW_H
