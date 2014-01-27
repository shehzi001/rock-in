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
#ifndef PLY_STRUCTURE
#define PLY_STRUCTURE

namespace Tracking{
  
struct PlyVertex {
	float x,y,z;                // spatial position
	float nx, ny, nz;           // normal vector
	float s, t;
	unsigned char r, g, b;      // color
};

struct PlyFace {
	unsigned short nverts;      // number of vertices used for the face (Quad=4, Triangle=3)
	unsigned int* v;            // pointer to memory holding the vertex-index list
	float t[3], b[3], n[3];			// Tangent space vectors
};

struct PlyEdge {
	unsigned short start;       // start vertex index
	unsigned short end;         // end vertex index
};

struct PlyPass {
	unsigned short nfaces;			// Number of faces using this pass
	unsigned int* f;						// pointer to memory holding the face-index list
	float m0,m1,m2,m3;					// matrix entries
	float m4,m5,m6,m7;					// matrix entries
	float m8,m9,m10,m11;					// matrix entries
	float m12,m13,m14,m15;					// matrix entries	
	float x,y,w,h;							// bounding box of texture with respect to modelview-projection-matrix
	unsigned short tex;					// index of texture
};

}

#endif
