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

#ifndef _HIST_DESC_POLAR_H_
#define _HIST_DESC_POLAR_H_

#include "tgHistogram.h"
#include "v4r/TomGine/tgModel.h"
#include "v4r/TomGine/tgMathlib.h"

namespace TomGine{

/** @brief Descriptor of image patch using histograms. */
class tgHistDesc2D
{
private:
//	bool ispow2(unsigned a);
	unsigned global2local(	unsigned idx,
						const TomGine::tgRect2Di &bb,
						const unsigned &subsizex, const unsigned &subsizey,
						const unsigned &nsegx,
						unsigned &subidx);
//	unsigned local2global(	unsigned idx,
//						const TomGine::tgRect2Di &bb,
//						const unsigned &subsize, const unsigned &subsizey,
//						const unsigned &segx, const unsigned &segy);


public:
	std::vector< TomGine::tgHistogram2D > m_histlist;
	std::vector< std::vector<TomGine::vec2> > m_subdata;
	std::vector< float > m_maskratio;
	TomGine::tgRect2Di m_bb;
	unsigned m_subsizex, m_subsizey;
	unsigned m_nsegx, m_nsegy;

	/** @brief preprocessing, calculate segment size */
	void Preprocess( const TomGine::tgRect2Di &bb );

public:

	tgHistDesc2D(unsigned segx, unsigned segy, unsigned hist_bins_x, unsigned hist_bins_y);

	/** @brief Calculates the histograms of the descriptor.
	 *  @param data		The data block of the image patch.	 */
	void Process(	const std::vector<TomGine::vec2> &data,
					const TomGine::tgRect2Di &bb,
					TomGine::vec2 min_val, TomGine::vec2 max_val);

	/** @brief Calculates the histograms of the descriptor with object mask.
	 *  @param data		The data block of the image patch.
	 *  @param mask		The mask indicating if a data element belongs to the object or not. (must be same size as data)*/
	void Process(	const std::vector<TomGine::vec2> &data,
					const TomGine::tgRect2Di &bb,
					TomGine::vec2 min_val, TomGine::vec2 max_val,
					const std::vector<bool> &mask, double vis_ratio=0.0 );

	/** @brief Inner product of histograms of two descriptors. */
	std::vector<double> operator*(const tgHistDesc2D& h) const;

	/** @brief Euclidean distance of histograms of two descriptors. */
	std::vector<double> Euclidean(const tgHistDesc2D& h, double vis_ratio=0.0) const;

	/** @brief Intersection of histograms of two descriptors. */
	std::vector<double> Intersect(const tgHistDesc2D& h, double vis_ratio=0.0) const;

	/** @brief Fidelity similarity of histograms of two descriptors. */
	std::vector<double> Fidelity(const tgHistDesc2D& h) const;

	/** @brief Get number of segments in x */
	inline unsigned GetSegX() { return m_nsegx; }
	/** @brief Get number of segments in y */
	inline unsigned GetSegY() { return m_nsegy; }

};

} // namespace TomGine

#endif
