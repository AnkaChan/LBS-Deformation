#ifndef _SMPL_MODEL_H_
#define _SMPL_MODEL_H_

#define EIGEN_STACK_ALLOCATION_LIMIT 100000000
#include "SkeletonModel.h"

#define SMPL_NUM_VERTS 6890
#define SMPL_NUM_JOINTS 24
#define SMPL_NUM_JOINTS_PARAM (3*SMPL_NUM_JOINTS) 

template <typename T>
using SMPLModel = SkeletonModel<SMPL_NUM_VERTS, SMPL_NUM_JOINTS,T>;

template <typename T>
using SMPLModelStaticData = SkeletonStaticData<SMPL_NUM_VERTS, SMPL_NUM_JOINTS, T>;

#endif // _SMPL_MODEL_H_
