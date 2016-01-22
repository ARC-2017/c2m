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

// �_�Q�̃Z���^�����O�DHPR�I�y���[�^�ɕK�v
// input (input)  ���͓_�Q
// offset(output) �I�t�Z�b�g��
// output(output) �I�t�Z�b�g��̓_�Q
//
void AkiCloudCentering( const pcl::PointCloud<pcl::PointNormal>::Ptr input,  pcl::PointXYZ *offset, 
						const pcl::PointCloud<pcl::PointNormal>::Ptr output );
void AkiCloudCentering( const pcl::PointCloud<pcl::PointXYZ>::Ptr input,  pcl::PointXYZ *offset, 
						const pcl::PointCloud<pcl::PointXYZ>::Ptr output );
// HPR�I�y���[�^
// input (input)  ���͓_�Q
// view_point (input) ���_
// param (input) �p�����[�^
// visibleIdx (output) �ϑ��_��ID
void HPR_operator( const pcl::PointCloud<pcl::PointNormal>::Ptr input, const double3 *view_point, double param, pcl::PointCloud<int> &visibleIdx );
void HPR_operator( const pcl::PointCloud<pcl::PointXYZ>::Ptr input, const double3 *view_point, double param, pcl::PointCloud<int> &visibleIdx );
void HPR_operator( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputXYZRGB, const double3 *view_point, double param, std::vector<int> &visibleIdx );


// Observability map�̗̈�m��
// nObserve (input) ���_��
// nPoints (input) ���f���̓_��
// om (output) Observability map�{��
void AkiCreateObservabilityMap( const int nObserve, const int nPoints, struct observability_map *om );

// Obserbability map�̓_�Q�ۑ�
// pnt (input) ���͓_�Q
// om (input) Observability map
// obsmappcdfname (input) �o�͓_�Q��
void AkiSaveObservabilityMapPCD( const pcl::PointCloud<pcl::PointNormal>::Ptr pnt, const observability_map *om, 
								 const char obsmappcdfname[] );

// �x�N�g���y�A�̉ϑ����̌v�Z
void AkiCalcObservabilityOfVectorPairs( std::vector<vector_pair>& vp, struct observability_map *om );

#endif