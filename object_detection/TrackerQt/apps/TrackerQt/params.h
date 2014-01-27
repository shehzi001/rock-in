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
#ifndef _TRACKER_QT_PARAMS_H_
#define _TRACKER_QT_PARAMS_H_

#include <QDialog>
#include "v4r/Tracker/Tracker.h"

namespace Ui {
class Params;
}

class Params : public QDialog
{
  Q_OBJECT
  
public:
  explicit Params(QWidget *parent = 0);
  ~Params();

  void get_parameters(int &cam_id, Tracking::Tracker::Parameter&params) const;
  void set_parameters(const int &cam_id, const Tracking::Tracker::Parameter &params);
  void save_parameters();
  bool parameters_changed() { return params_changed; }

signals:
  void send_tracking_parameters(Tracking::Tracker::Parameter params);
  void send_sensor_parameters(double width, double height, int cam_id);
  
private slots:
  void on_ButtonOk_pressed();

  void on_pushApply_pressed();

  void on_pushCancel_pressed();

  void on_pushLoad_pressed();

  void on_pushSave_pressed();

private:
  Ui::Params *ui;
  bool params_changed;
};

#endif // PARAMS_H
