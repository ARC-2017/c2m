//////////////////////////////////////////////////////////////////////////////
//
//	libakiHPRobservabilitylib.h:
//
//  Shuichi AKIZUKI
//
//	(C) 2015 ISL, Chukyo University All rights reserved.
//
//  Note:
//		2015.01.30
//			
//////////////////////////////////////////////////////////////////////////////
#ifndef INCLUDE_libakiHPRobservabilitylib_h_
#define INCLUDE_libakiHPRobservabilitylib_h_

#include "common.h"
#include "common2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

// 点群のセンタリング．HPRオペレータに必要
// input (input)  入力点群
// offset(output) オフセット量
// output(output) オフセット後の点群
//
void AkiCloudCentering( const pcl::PointCloud<pcl::PointNormal>::Ptr input,  pcl::PointXYZ *offset, 
						const pcl::PointCloud<pcl::PointNormal>::Ptr output );
void AkiCloudCentering( const pcl::PointCloud<pcl::PointXYZ>::Ptr input,  pcl::PointXYZ *offset, 
						const pcl::PointCloud<pcl::PointXYZ>::Ptr output );
// HPRオペレータ
// input (input)  入力点群
// view_point (input) 視点
// param (input) パラメータ
// visibleIdx (output) 可観測点のID
void HPR_operator( const pcl::PointCloud<pcl::PointNormal>::Ptr input, const double3 *view_point, double param, pcl::PointCloud<int> &visibleIdx );
void HPR_operator( const pcl::PointCloud<pcl::PointXYZ>::Ptr input, const double3 *view_point, double param, pcl::PointCloud<int> &visibleIdx );
void HPR_operator( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputXYZRGB, const double3 *view_point, double param, std::vector<int> &visibleIdx );


// Observability mapの領域確保
// nObserve (input) 視点数
// nPoints (input) モデルの点数
// om (output) Observability map本体
void AkiCreateObservabilityMap( const int nObserve, const int nPoints, struct observability_map *om );

// Obserbability mapの点群保存
// pnt (input) 入力点群
// om (input) Observability map
// obsmappcdfname (input) 出力点群名
void AkiSaveObservabilityMapPCD( const pcl::PointCloud<pcl::PointNormal>::Ptr pnt, const observability_map *om, 
								 const char obsmappcdfname[] );

// ベクトルペアの可観測率の計算
void AkiCalcObservabilityOfVectorPairs( std::vector<vector_pair>& vp, struct observability_map *om );

#endif