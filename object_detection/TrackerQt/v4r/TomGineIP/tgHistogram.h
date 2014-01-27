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
 
#ifndef _TG_HISTOGRAM_H_
#define _TG_HISTOGRAM_H_

#include <vector>
#include <math.h>
#include <stdio.h>
#include <GL/gl.h>

#include "v4r/TomGine/tgMathlib.h"

namespace TomGine{

/** @brief Evaluate histogram of data. */
class tgHistogram
{
protected:
	std::vector<unsigned> m_counter;
	std::vector<double> m_pdf;
	double m_max;
	unsigned m_sum;
	unsigned m_size;

public:
	/** @brief Creates the Histogram
	 *  @param size	number of bins used. */
	tgHistogram(unsigned size);

	/** @brief Sets the counter of the histogram bins to 0. */
	void Reset();

	/** @brief Inner product of two histograms. */
	double operator*(const tgHistogram& h) const;

	/** @brief Euclidean distance of two histograms. */
	double Euclidean(const tgHistogram& h) const;

	/** @brief Intersection of two histograms. */
	double Intersect(const tgHistogram& h) const;

	/** @brief Fidelity similarity of two histograms. */
	double Fidelity(const tgHistogram& h) const;

	/** @brief Evaluates histogram of data given by clamping to the range [0,1],
	 * 		multiplying by the size of the histogram, and rounding to the nearest integer.
	 * 	@param data	The data to calculate the histogram from
	 * 	@param min	The minimum of the histogram range
	 * 	@param max	The maximum of the histogram range	 */
	void Evaluate(const std::vector<double> &data, const double &min, const double &max);

	/** @brief Returns the histogram counters. */
	std::vector<unsigned> GetCounter() const{ return m_counter; }

	/** @brief Returns the histogram normalized to its sum. */
	std::vector<double> GetNormalized() const{ return m_pdf; }

	/** @brief Get maximum number in histogram. */
	double GetMax() const{ return m_max; }

	/** @brief Get sum of histogram. */
	unsigned GetSum() const{ return m_sum; }

	/** @brief Returns the number of bins of the histogram. */
	unsigned Size() const{ return m_size; }

	/** @brief Prints the histogram counter to console. */
	void Print() const;

};

/** @brief Evaluate histogram with data given in polar coordinates (angle, magnitude). */
class tgHistogram2D : public tgHistogram
{
protected:
	unsigned m_size_x, m_size_y;
public:
	/** @brief Creates the Histogram
	 *  @param size	number of bins used. */
	tgHistogram2D(unsigned size_x, unsigned size_y) : tgHistogram(size_x * size_y)
	{
		m_size_x = size_x;
		m_size_y = size_y;
	}

	/** @brief Evaluates histogram of 2d-vectors, where a vector is given in polar coordinates,
	 * 			i.e. by its angle (x-component) and magnitude (y-component).
	 * 	@param data	The data to calculate the histogram from
	 * 	@param min	The minimum of the histogram range
	 * 	@param max	The maximum of the histogram range	 */
	void EvaluatePolar(const std::vector<vec2> &data, const vec2 &min, const vec2 &max);

};

}

#endif
