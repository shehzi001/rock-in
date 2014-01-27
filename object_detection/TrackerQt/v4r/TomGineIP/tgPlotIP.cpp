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
#include "tgPlotIP.h"

using namespace TomGine;

tgPlotIP::tgPlotIP(int x, int y, unsigned w, unsigned h)
{
  Define(x, y, w, h);
  m_fontcolor = vec3(0.5f, 0.5f, 0.5f);
  m_axiscolor = vec3(0.5f, 0.5f, 0.5f);
  m_gridcolor = vec3(0.5f, 0.5f, 0.5f);

  m_y1_color = vec3(1.0f, 1.0f, 0.0f);
  m_y2_color = vec3(1.0f, 0.0f, 1.0f);
  m_y3_color = vec3(0.0f, 1.0f, 1.0f);
  m_y4_color = vec3(0.0f, 1.0f, 0.0f);
}

void tgPlotIP::DrawHistogram(const std::vector<double> &data, const double &max_val) const
{
  float fx = float(x);
  float fy = float(y);

  float s = this->y_scale / max_val;

  glBegin(GL_LINES);

  float dx = float(w) / data.size();
  for( unsigned i = 0; i < data.size(); i++ )
  {
    vec3 rgb = hsv2rgb(vec3(360.0 * (dx * i) / h, 1.0, 1.0));
    glColor3f(rgb.x, rgb.y, rgb.z);
    float dy = s * data[i];
    glVertex3f(fx + dx * i, fy, 0.0f);
    glVertex3f(fx + dx * i, fy + dy, 0.0f);
  }

  //            dx = float(w) / m_buffer_size;
  //
  //            for(unsigned i=0; i<m_buffer_size; i++)
  //            {
  //                    vec3 rgb = hsv2rgb( vec3(360.0 * (dx*i) / h, 1.0, 1.0) );
  //                    glColor3f(rgb.x, rgb.y, rgb.z);
  //                    glVertex3f(fx+dx*i, y-0.02f*h, 0.0f);
  //                    glVertex3f(fx+dx*i, y-0.1f*h, 0.0f);
  //            }

  glEnd();

}
