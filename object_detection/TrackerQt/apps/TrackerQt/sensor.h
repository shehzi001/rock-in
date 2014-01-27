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
#ifndef _TRACKER_QT_SENSOR_H_
#define _TRACKER_QT_SENSOR_H_

#include <QThread>
#include <QMutex>
#include <opencv2/opencv.hpp>
#include <qrect.h>

class Sensor : public QThread
{
  Q_OBJECT

public:
  Sensor(double width, double height, int cam_id);
  ~Sensor();

  void stop();
  bool is_running();
  void get_image(cv::Mat& img);
  QRect geometry();

signals:
  void new_image(cv::Mat img);

public slots:
  void set_parameter(double width, double height, int cam_id);

private:
  void run();

  bool m_run;
  double m_width;
  double m_height;
  int m_cam_id;

  cv::Mat m_img;

  QMutex mutex;

};

#endif // SENSOR_H
