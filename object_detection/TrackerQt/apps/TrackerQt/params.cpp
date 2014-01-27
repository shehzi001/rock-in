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
#include "params.h"
#include "ui_params.h"
#include <QFileDialog>
#include <fstream>

Params::Params(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::Params),
  params_changed(false)
{
  ui->setupUi(this);
}

Params::~Params()
{
  delete ui;
}

void Params::get_parameters(int &cam_id, Tracking::Tracker::Parameter& params) const
{
  // Camera
  cam_id = ui->cam_id->text().toInt();
  params.camPar.width = ui->width->text().toUInt();
  params.camPar.height = ui->height->text().toUInt();
  params.camPar.fx = ui->fx->text().toFloat();
  params.camPar.fy = ui->fy->text().toFloat();
  params.camPar.cx = ui->cx->text().toFloat();
  params.camPar.cy = ui->cy->text().toFloat();
  params.camPar.k1 = ui->k1->text().toFloat();
  params.camPar.k2 = ui->k2->text().toFloat();
  params.camPar.k3 = ui->k3->text().toFloat();
  params.camPar.p1 = ui->p1->text().toFloat();
  params.camPar.p2 = ui->p2->text().toFloat();
  params.camPar.rot[0] = ui->r11->text().toFloat();
  params.camPar.rot[1] = ui->r21->text().toFloat();
  params.camPar.rot[2] = ui->r31->text().toFloat();
  params.camPar.rot[3] = ui->r12->text().toFloat();
  params.camPar.rot[4] = ui->r22->text().toFloat();
  params.camPar.rot[5] = ui->r32->text().toFloat();
  params.camPar.rot[6] = ui->r13->text().toFloat();
  params.camPar.rot[7] = ui->r23->text().toFloat();
  params.camPar.rot[8] = ui->r33->text().toFloat();
  params.camPar.pos.x = ui->tx->text().toFloat();
  params.camPar.pos.y = ui->ty->text().toFloat();
  params.camPar.pos.z = ui->tz->text().toFloat();

  float dpi = M_PI / 180.0f;

  // Edge detection
  params.edge_tolerance = ui->edge_matching_tol->text().toFloat() * dpi;
  params.minTexGrabAngle = ui->min_tex_grab_angle->text().toFloat() * dpi;
  params.num_spreadings = ui->num_of_spreadings->text().toUInt();
  params.max_kernel_size = ui->max_kernel_size->text().toUInt();
  params.model_sobel_th = ui->model_soble_th->text().toFloat();
  params.image_sobel_th = ui->image_soble_th->text().toFloat();

  if(ui->radioEdge->isChecked())
    params.method = Tracking::EDGE;
  else if(ui->radioColor->isChecked())
    params.method = Tracking::COLOR;
  else if(ui->radioEdgeColor->isChecked())
    params.method = Tracking::EDGECOLOR;

  // Particle filtering
  params.variation.r.x = ui->rot_var->text().toFloat() * dpi;
  params.variation.r.y = ui->rot_var->text().toFloat() * dpi;
  params.variation.r.z = ui->rot_var->text().toFloat() * dpi;
  params.variation.t.x = ui->trans_var->text().toFloat();
  params.variation.t.y = ui->trans_var->text().toFloat();
  params.variation.t.z = ui->trans_var->text().toFloat();
  params.variation.z = ui->zoom_var->text().toFloat();
  params.num_recursions = ui->iterations->text().toInt();
  params.num_particles = ui->num_particles->text().toInt();
  params.convergence = ui->convergence->text().toInt();
  params.pred_no_convergence = ui->non_converging->text().toFloat();
  params.keep_best_particles = ui->non_moving->text().toFloat();
  params.lpf = ui->lpf_factor->text().toFloat();

  //  params.shaderPath = ui->shader_path->text().toStdString();
  params.modelPath = ui->model_path->text().toStdString();
}

void Params::set_parameters(const int &cam_id, const Tracking::Tracker::Parameter& params)
{
  // Camera
  ui->cam_id->setText(QString::number(cam_id));
  ui->width->setText(QString::number(params.camPar.width));
  ui->height->setText(QString::number(params.camPar.height));
  ui->fx->setText(QString::number(params.camPar.fx));
  ui->fy->setText(QString::number(params.camPar.fy));
  ui->cx->setText(QString::number(params.camPar.cx));
  ui->cy->setText(QString::number(params.camPar.cy));
  ui->k1->setText(QString::number(params.camPar.k1));
  ui->k2->setText(QString::number(params.camPar.k2));
  ui->k3->setText(QString::number(params.camPar.k3));
  ui->p1->setText(QString::number(params.camPar.p1));
  ui->p2->setText(QString::number(params.camPar.p2));
  ui->r11->setText(QString::number(params.camPar.rot[0]));
  ui->r21->setText(QString::number(params.camPar.rot[1]));
  ui->r31->setText(QString::number(params.camPar.rot[2]));
  ui->r12->setText(QString::number(params.camPar.rot[3]));
  ui->r22->setText(QString::number(params.camPar.rot[4]));
  ui->r32->setText(QString::number(params.camPar.rot[5]));
  ui->r13->setText(QString::number(params.camPar.rot[6]));
  ui->r23->setText(QString::number(params.camPar.rot[7]));
  ui->r33->setText(QString::number(params.camPar.rot[8]));
  ui->tx->setText(QString::number(params.camPar.pos.x));
  ui->ty->setText(QString::number(params.camPar.pos.y));
  ui->tz->setText(QString::number(params.camPar.pos.z));

  float dpi = 180.0f / M_PI;

  // Edge detection
  ui->edge_matching_tol->setText(QString::number(params.edge_tolerance * dpi));
  ui->min_tex_grab_angle->setText(QString::number(params.minTexGrabAngle * dpi));
  ui->num_of_spreadings->setText(QString::number(params.num_spreadings));
  ui->max_kernel_size->setText(QString::number(params.max_kernel_size));
  ui->model_soble_th->setText(QString::number(params.model_sobel_th));
  ui->image_soble_th->setText(QString::number(params.image_sobel_th));

  if(params.method == Tracking::EDGE)
    ui->radioEdge->click();
  else if(params.method == Tracking::COLOR)
    ui->radioColor->click();
  else if(params.method == Tracking::EDGECOLOR)
    ui->radioEdgeColor->click();

  // Particle filtering
  ui->rot_var->setText(QString::number(params.variation.r.x * dpi));
  ui->rot_var->setText(QString::number(params.variation.r.y * dpi));
  ui->rot_var->setText(QString::number(params.variation.r.z * dpi));
  ui->trans_var->setText(QString::number(params.variation.t.x));
  ui->trans_var->setText(QString::number(params.variation.t.y));
  ui->trans_var->setText(QString::number(params.variation.t.z));
  ui->zoom_var->setText(QString::number(params.variation.z));
  ui->iterations->setText(QString::number(params.num_recursions));
  ui->num_particles->setText(QString::number(params.num_particles));
  ui->convergence->setText(QString::number(params.convergence));
  ui->non_converging->setText(QString::number(params.pred_no_convergence));
  ui->non_moving->setText(QString::number(params.keep_best_particles));
  ui->lpf_factor->setText(QString::number(params.lpf));

  ui->model_path->setText(QString::fromStdString(params.modelPath));
}

void Params::save_parameters()
{
  QString filename = QFileDialog::getSaveFileName(this, tr("Save parameter file..."), "tracking.conf", tr("*.conf"));

  if(filename.size()==0)
    return;

  std::ofstream out(filename.toStdString().c_str());

  if(!out.is_open())
    return;

  int cam_id;
  Tracking::Tracker::Parameter params;
  get_parameters(cam_id, params);

  out << "# Camera ID" << std::endl;
  out << cam_id << std::endl;
  out << params;

  out.close();
  params_changed = false;
}

void Params::on_ButtonOk_pressed()
{
  int cam_id;
  Tracking::Tracker::Parameter params;

  get_parameters(cam_id, params);

  emit send_tracking_parameters(params);
  emit send_sensor_parameters((double)params.camPar.width, (double)params.camPar.height, cam_id);
  params_changed = true;

  this->hide();
}

void Params::on_pushApply_pressed()
{
  int cam_id;
  Tracking::Tracker::Parameter params;

  get_parameters(cam_id, params);

  emit send_tracking_parameters(params);
  emit send_sensor_parameters((double)params.camPar.width, (double)params.camPar.height, cam_id);
  params_changed = true;
}

void Params::on_pushCancel_pressed()
{
  this->hide();
}

void Params::on_pushLoad_pressed()
{
  QString filename = QFileDialog::getOpenFileName(this,tr("Open file..."),"",tr("*.conf"));

  if(filename.size()==0)
    return;

  std::ifstream in(filename.toStdString().c_str());

  if(!in.is_open())
    return;

  int cam_id;
  Tracking::Tracker::Parameter params;

  std::string linebuffer;
  getline(in,linebuffer);
  in >> cam_id;
  in.get();
  in >> params;

  params.print();

  set_parameters(cam_id, params);

  in.close();
}

void Params::on_pushSave_pressed()
{
  save_parameters();
}
