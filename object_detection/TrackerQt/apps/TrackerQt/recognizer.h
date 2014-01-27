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
#ifndef _TRACKER_QT_RECOGNIZER_H_
#define _TRACKER_QT_RECOGNIZER_H_

#include <QThread>
#include <QMutex>
#include "v4r/Recognizer3D/Recognizer3D.h"
#include "v4r/Tracker/Tracker.h"

class Recognizer : public QThread
{
  Q_OBJECT

  enum Command{
    LEARN = 0,
    RECOGNIZE = 1
  };

signals:
  void start_call();

public:
  Recognizer(const Tracking::Tracker::Parameter& params);
  ~Recognizer();

  void learn(const cv::Mat& img, const TomGine::tgModel& model, const TomGine::tgPose& pose);
  void recognize(const cv::Mat& img, TomGine::tgPose& pose, float &confidence);
  void load(const std::string& filename);
  void save(const std::string& filename);

  void run();

  blortRecognizer::Recognizer3D* m_recognizer;
  Command cmd;
  TomGine::tgModel m_model;
  TomGine::tgPose m_cam_pose, m_pose;
  cv::Mat m_img;
  float m_confidence;

  QMutex mutex;

};

#endif
