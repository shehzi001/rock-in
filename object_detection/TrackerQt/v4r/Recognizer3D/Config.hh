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
 * $Id: Config.hh,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 */

#ifndef P_CONFIG_HH
#define P_CONFIG_HH

#include <stdlib.h>
#include <string>
#include <map>
#include "PNamespace.hh"

namespace P 
{

class Config
{
public:
  map<string, string> items;

public:
  Config() {}
  Config(const char *filename);
  void Load(const char *filename);
  void AddItem(const string &name, const string &value) {items[name] = value;}
  bool ExistItem(const string &name);
  string GetValueString(const string &name) {return items[name];}
  int GetValueInt(const string &name) {return atoi(items[name].c_str());}
  double GetValueDouble(const string &name) {return atof(items[name].c_str());}
};

}

#endif

