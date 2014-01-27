/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2013, Johann Prankl
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
 * @author johann.prankl
 *
 */
/**
 * $Id$
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_CODEBOOK_ENTRY_HH
#define P_CODEBOOK_ENTRY_HH

#include <limits.h>
#include "PNamespace.hh"
#include "Array.hh"
#include "KeypointDescriptor.hh"

namespace P
{

class KeypointDescriptor;

class CodebookEntry
{
public:
  float sqr_sigma;
  KeypointDescriptor *model;
  P::Array<KeypointDescriptor* > occurrences;

  bool good;          //-1 false match, 0 false and correct match, 1 only correct match
  bool bad;
  int cntGood;       // count good matches
  int cntTime;       // ''timestamp''
  double reliability;

  CodebookEntry();
  CodebookEntry(KeypointDescriptor *k);
  ~CodebookEntry();

  inline unsigned Size(){return occurrences.Size();}
  inline void Clear();
  inline KeypointDescriptor *Insert(KeypointDescriptor* oc);
  inline bool Insert(KeypointDescriptor* occ, float sqr_sigma); 
  inline void ComputeModel();
  inline float CombinedSqrSigma(CodebookEntry *c);
  inline float CombinedSqrSigma(KeypointDescriptor *occ);
  inline bool Combine(CodebookEntry *c);
  inline float DistanceSqr(CodebookEntry *c);

};

inline void DeleteCodebook(Array<CodebookEntry*> &codebook);

}

#include "CodebookEntry.ic"

#endif

