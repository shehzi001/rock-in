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
 * @file ModelLoader.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Loading geometrical models from file
 * @namespace Tracking
 */
 
#ifndef _MODELLOADER_H_
#define _MODELLOADER_H_

#include <stdlib.h>
#include <stddef.h>
#include <string>
#include "v4r/TomGine/tgModel.h"

#include "ply.h"
#include "PlyStructure.h"
#include "TrackerModel.h"


namespace Tracking{

/** @brief Loading geometrical models from file */
class ModelLoader
{
private:

	bool propertyIsInList(PlyProperty* prop, PlyProperty* list, int n, int* index);

public:
	ModelLoader();
	~ModelLoader();
	
	/** @brief Loads PLY (Polygon File Format, Stanford Triangle Format) 
	*		@param Model Storage for model
	*		@param filename path and filename of file to load
	*		@return true on success, false on failure
	*/
  bool LoadPly(TomGine::tgModel &model, const char* filename);
  bool LoadPly(TrackerModel &model, const char* filename);
	
  bool SavePly(TomGine::tgModel &model, const char* filename);
  bool SavePly(TrackerModel &model, const char* filename);
	   
};

} // namespace Tracking

#endif
