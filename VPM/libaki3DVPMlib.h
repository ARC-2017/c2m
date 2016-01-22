//////////////////////////////////////////////////////////////////////////////
//
//	libaki3DVPMlib.h: Function for 3-D Vector Pair Matching.
//
//  Shuichi AKIZUKI
//
//	(C) 2012 ISL, Chukyo University All rights reserved.
//
//			
//////////////////////////////////////////////////////////////////////////////
#ifndef INCLUDE_libaki3DVPMlib_h_
#define INCLUDE_libaki3DVPMlib_h_

#include "common.h"
#include "common2.h"
#include "aki3DVPM.h"
#include <vector>
//Point Cloud Library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>


#include "libPCLFunclib.h"
#include "libakitimerlib.h"
#include "libakiHPRobservabilitylib.h"

void AkiCreateInt3D( int isx, int isy, int isz, aki_int_3d *II );

void AkiClearInt3D( aki_int_3d *II, int value );

inline void AkiCalcLength( double3 v, double *length );

void AkiCalcDistance( double3 v1, double3 v2, double *dist );

void AkiNormalize( const struct double3 v, double3 *normalized );

inline void AkiCrossProduct( double3 *va, double3 *vb, double3 *vc );

inline void AkiInnerProduct( double3 va, double3 vb, double *ip );

inline void Aki3DMatrixx3DPoint( double rot[][3], double3 v, double3 *output );

inline void AkiMatrixxMatrix3D( double a[][3], double b[][3], double ans[][3] );

void AkiCalcInverseMatrix3D( double mat[][3], double matinv[][3] );

void AkiCalcInverseMatrixN( double mat[][3], double inv[][3] );

inline void AkiMakeTransposedMatrix3D( double mat[][3], double trans[][3] );

void AkiCopyVectorPair( vector_pair *vp1, vector_pair *vp2 );

inline void AkiInitializeVectorPair( vector_pair *vp, vector_pair *ivp );

void AkiShowVectorPair( vector_pair *vp );

void AkiClearVectorPair( vector_pair *vp );

inline void AkiVectorPair2Matrix3D( vector_pair *vp, double mat[][3] );

void AkiShowMatrix3D( double mat[][3] );
	
inline void AkiCalcRotationalMatrixFrom2VectorPairs( vector_pair *vpM, vector_pair *vpS, double matR[][3] ); 

void AkiVectorPairOrthonormalization( vector_pair *vp, vector_pair *vpo );

void AkiCalcOccurrenceProbabilityOfVectorPair( std::vector<vector_pair>& vp, struct aki_int_3d *HH3D );

void AkiNormalizeProbabilityOfVectorPair( std::vector<vector_pair>& vp );

void AkiIntegrationOccurenceProbabilityAndObservability( std::vector<vector_pair>& vp );

void AkiIntegrationOccurenceProbability( std::vector<vector_pair>& vp );

void AkiIntegrationObservability( std::vector<vector_pair>& vp );

void AkiCombSortVectorPair( std::vector<vector_pair>& vp );

void AkiRotateMatrix2EulerAngleXYZ( double mat[][3], double *EulerX, double *EulerY, double *EulerZ );

void AkiEulerAngleXYZ2RotateMatrix( double EulerX, double EulerY, double EulerZ, double mat[][3] );

void AkiCreateVotingSpaceTrans( int sx, int sy, int sz, int nVP, VotingSpaceTrans *VSTrans );

void AkiClearVotingSpaceTrans( VotingSpaceTrans *VSTrans );

void AkiRot2RollPitchYaw(double rot[][3],double *roll,double *pitch, double *yaw);

void AkiRollPitchYaw2Rot(double roll, double pitch, double yaw, double rot[][3]);

void CombSort( const std::vector<float>& values, std::vector<int>& Idx );

void AkiVectorPairRandomReduce( std::vector<vector_pair>& vp, const int n_reduce );

bool RecgVPM( int *itemIdx, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
			  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge, double *VPMworkPos, double *VPMScore, 
			  int *CenterIdx, std::vector<int>& cloud_cluster_idx );

#endif