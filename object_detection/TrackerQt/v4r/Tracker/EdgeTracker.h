/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2013, Thomas Mörwald
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
 /**
 * @file EdgeTracker.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Tracker using geometry edges for matching.
 * @namespace Tracker
 */
#ifndef EDGE_TRACKER_H
#define EDGE_TRACKER_H

#include "Tracker.h"

namespace Tracking{

/** @brief class EdgeTracker */
class EdgeTracker : public Tracker
{
private:
	
	// Resources
	TomGine::tgShader* m_shadeEdgeCompare;

	// Functions
// 	void particle_processing(TrackerModel* model, TomGine::tgShader* shadeCompare);
	void particle_filtering(ModelEntry* modelEntry);
	
public:
	EdgeTracker();
	
	virtual bool initInternal();
	
	virtual void image_processing(unsigned char* image, GLenum format=GL_BGR);
	virtual void image_processing(unsigned char* image, const TomGine::tgModel &m, const TomGine::tgPose &p, GLenum format=GL_BGR);
	virtual void image_processing(unsigned char* image, int model_id, const TomGine::tgPose &p, GLenum format=GL_BGR){};
	
	virtual bool track();
	virtual bool track(int id);
						
	virtual void drawResult(float linewidth=1.0f);

};

} // namespace Tracking

#endif
