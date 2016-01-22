#ifndef INCLUDE_libsprhlib_h_
#define INCLUDE_libsprhlib_h_

#include "common.h"
#include "common2.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

//AkiLib
#include "akidata.h"
#include "aki3DVPM.h"
#include "libaki3DVPMlib.h"
#include "libHSVlib.h"
#include "libPCLFunclib.h"
#include "libakiHPRobservabilitylib.h"
#include "sprh.h"
#include "libsprhlib.h"
#include "libakitimerlib.h"
#include "ch.h"
#include "libchlib.h"

// 特徴量の初期化
void InitializeSPRH( const int dist_max, const int angle_max, const int dist_pitch, const int angle_pitch, const int n_sampling, struct SPRH *sprh );

// 特徴量の読み込み
bool LoadSPRH( char name[], struct SPRH *sprh );

// 特徴量の算出
void CreateSPRH( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud, const int area, const double sampling_rate, struct SPRH *sprh );

// 特徴量の可視化
void SaveSPRHAsImg( char name[], struct SPRH *sprh );

// 特徴量の保存
void SaveSPRH( char name[], struct SPRH *sprh ); 

// 視点の設定．view_pitchは緯度方向の角度ピッチ
void SetViewpoint( int view_pitch, std::vector<double3> *viewpoint );

// SPRH間の類似度計算
//void MatchingSPRH( const struct SPRH *sprh1, const struct SPRH *sprh2, double *score ); 

void MatchingSPRH2( const struct SPRH *sprh_m, const struct SPRH *sprh_s, double *score ); 

bool RecgSimple( const int num_of_seg, int *itemIdx, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double *SimpleworkPos, double *SimpleScore,
				 int *CenterIdx, std::vector<int>& cloud_cluster_idx );

void extracteCluster( pcl::PointCloud<pcl::PointNormal>::Ptr input, int num_of_object, int max_it, float th_var, std::vector<std::vector<int>>& segIdx );

#endif