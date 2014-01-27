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


#include "ModelObject3D.hh"
#include <stdexcept>




namespace P 
{


/********************** ModelObject3D ************************
 * Constructor/Destructor
 */
ModelObject3D::ModelObject3D()
{
}


ModelObject3D::~ModelObject3D()
{
}

/**
 * Create new object model
 */
void ModelObject3D::ComputeNewHnorm(P::Array<KeypointDescriptor*> &keys, double Hnorm[9])
{
  if (keys.Size()==0)
    return;

  Hnorm[0] = 1., Hnorm[1] =  0., Hnorm[2] = 0.;
  Hnorm[3] = 0., Hnorm[4] =  1., Hnorm[5] = 0.;
  Hnorm[6] = 0., Hnorm[7] =  0., Hnorm[8] = 1.;

  for (unsigned i=0; i<keys.Size(); i++)
  {
    Hnorm[2] -= keys[i]->p.x;
    Hnorm[5] -= keys[i]->p.y;
  }

  Hnorm[2] /= (double)keys.Size();
  Hnorm[5] /= (double)keys.Size();
}

/**
 * Insert a new keypoint to the codebook
 */
void ModelObject3D::InsertMeanShift(Array<KeypointDescriptor* > &keys, P::Array<CodebookEntry*> &codebook, double H[9])
{
  double scale, angle;
  P::Vector2 center;
  KeypointDescriptor *occ;
  float sigma;
  bool inserted;
  Vector2 p;

  for (unsigned i=0; i<codebook.Size(); i++)
    codebook[i]->cntTime++;
  
  for (unsigned i=0; i<keys.Size(); i++)
  {
    inserted=false;
    for (unsigned j=0; j<codebook.Size(); j++)
    {
      if (((KeypointDescriptor*)keys[i])->Match(codebook[j]->model) != FLT_MAX)
      {
        sigma = codebook[j]->CombinedSqrSigma((KeypointDescriptor*)keys[i]);
        if (sigma < codebook[j]->model->GetThr()*Def::DO_CLUSTER_SIGMA_THR)
        {
          codebook[j]->Insert((KeypointDescriptor*)keys[i]);
          occ=codebook[j]->occurrences.Last();
          GetPosScaleAngle(occ->p.x, occ->p.y,H, center.x,center.y,scale,angle);
          p = occ->p;
          MapPoint(&p.x,H, &occ->p.x);
          occ->scale *= scale;
          occ->angle += ScaleAngle_mpi_pi(angle);
          if (codebook[j]->cntTime > Def::DO_TIME_MEAN) codebook[j]->cntTime = Def::DO_TIME_MEAN;
          inserted=true;
        }
      }
    }
    if (!inserted)
    {
      codebook.PushBack(new CodebookEntry((KeypointDescriptor*)keys[i]));
      occ = codebook.Last()->occurrences[0];
      GetPosScaleAngle(occ->p.x, occ->p.y,H, center.x,center.y,scale,angle);
      p = occ->p;
      MapPoint(&p.x,H, &occ->p.x);
      occ->scale *= scale;
      occ->angle += ScaleAngle_mpi_pi(angle);
    }
  }
}






/******************************** PUBLIC **************************/

/**
 * Create new object model
 */
void ModelObject3D::AddToModel(Array<KeypointDescriptor *> &keys, Object3D &obj)
{
  double H[9];

  ComputeNewHnorm(keys, H);
  InsertMeanShift(keys, obj.codebook, H);
}


/**
 * Save 3d object model
 */
void ModelObject3D::SaveModel(const char *filename, Object3D &obj)
{
  CodebookEntry *cbe;

  ofstream out(filename);

  if(!out)
    throw Except(__HERE__,"Error opening file!");

  out<<obj.id<<'\n';

  out<<obj.codebook.Size()<<'\n';
  for (unsigned i=0; i<obj.codebook.Size(); i++)
  {
    cbe = obj.codebook[i];
    out<<cbe->sqr_sigma<<'\n';
    out<<cbe->cntGood<<'\n';
    out<<cbe->cntTime<<'\n';
    out<<cbe->reliability<<'\n';

    cbe->model->SaveAll(out,*cbe->model);

    out<<cbe->occurrences.Size()<<'\n';
    for (unsigned j=0; j<cbe->occurrences.Size(); j++)
      cbe->occurrences[j]->SaveAll(out,*cbe->occurrences[j]);
  }

  out.close();
}

/**
 * Load 3d object model
 */
bool ModelObject3D::LoadModel(const char *filename, Object3D &obj)
{
  unsigned cbSize, ocSize;
  KeypointDescriptor *oc;
  CodebookEntry *cbe;

  DeleteCodebook(obj.codebook);

  ifstream in(filename);

  if(!in){
    char errmsg[128];
    sprintf(errmsg, "[ModelObject3D::LoadModel] Cannot open file '%s'", filename);
    throw std::runtime_error(errmsg);
  }

  in>>obj.id;

  in>>cbSize;
  for (unsigned i=0; i<cbSize; i++)
  {
    cbe = new CodebookEntry();

    in>>cbe->sqr_sigma;
    in>>cbe->cntGood;
    in>>cbe->cntTime;
    in>>cbe->reliability;

    cbe->model = new KeypointDescriptor();
    cbe->model->LoadAll(in, *cbe->model);

    in>>ocSize;

    for (unsigned j=0; j<ocSize; j++)
    {
      oc = new KeypointDescriptor();
      oc->LoadAll(in, *oc);
      cbe->occurrences.PushBack(oc);
    }

    obj.codebook.PushBack(cbe);
  }

  in.close();
  return true;
}






/************************* DEBUGGING METHODES *****************************/




}  // -- THE END --

