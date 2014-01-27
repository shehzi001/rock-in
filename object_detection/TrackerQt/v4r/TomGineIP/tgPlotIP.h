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

#ifndef TOMGINE_TGPLOT_IP
#define TOMGINE_TGPLOT_IP

#include "v4r/TomGine/tgPlot2D.h"

#include "tgHistogram.h"

namespace TomGine
{

  class tgPlotIP : public tgPlot2D
  {
  public:
    tgPlotIP(int x=0, int y=0, unsigned w=100, unsigned h=100);

    /** @brief Draws histogram as bar diagram.
     *  @param data The histogram pdf - tgHistogram::GetNormalized()
     *  @param max_val The maximum probability occuring in the histogram pdf - tgHistogram::GetMax() */
    void
    DrawHistogram (const std::vector<double> &data, const double &max_val) const;

    /** @brief Draws histogram as bar diagram. */
    void
    DrawHistogram (const TomGine::tgHistogram &hist) const
    {
      DrawHistogram (hist.GetNormalized (), hist.GetMax ());
    }

  };

}

#endif
