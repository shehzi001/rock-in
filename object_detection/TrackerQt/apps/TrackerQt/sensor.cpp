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
#include "sensor.h"
#include <stdexcept>

using namespace std;

Sensor::Sensor(double width, double height, int cam_id) :
  m_width(width), m_height(height), m_cam_id(cam_id)
{

}

Sensor::~Sensor()
{
  stop();
}

void Sensor::stop()
{
  if(m_run)
  {
    m_run = false;
    this->wait();
  }
}

bool Sensor::is_running()
{
  return m_run;
}

void Sensor::get_image(cv::Mat& img)
{
  if(!m_run)
    return;

  QMutexLocker ml(&mutex);
  m_img.copyTo(img);
}

QRect Sensor::geometry()
{
  return QRect(0,0,m_width,m_height);
}

void Sensor::set_parameter(double width, double height, int cam_id)
{
  if(m_cam_id == cam_id && m_width == width && m_height == height)
    return;

  if(m_run)
  {
    this->stop();
    this->wait();
  }

  m_cam_id = cam_id;
  m_width = width;
  m_height = height;

  this->start();
}

void Sensor::run()
{
  m_run = true;

  cv::VideoCapture cap(m_cam_id);
  if(!cap.isOpened())
  {
    cout << "[Sensor::run()] Error, can not start capturing device " << m_cam_id << endl;
    return;
  }

  cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);

  double w = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  double h = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

  if(w!=m_width || h!=m_height)
  {
    cout << "[Sensor::run()] Warning, sensor resolution is " << w << "x" << h <<
            " although " << m_width << "x" << m_height << " is set." << endl;
    throw std::runtime_error("[Sensor::run()] Video resolution not supported.");
  }

  while(m_run)
  {
    {
      if(cap.grab())
      {
        QMutexLocker ml(&mutex);
        cap.retrieve(m_img);
      }
    }
    emit new_image(m_img);
  }

  cap.release();
}
