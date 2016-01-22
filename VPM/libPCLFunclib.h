/////////////////////////////////////////////////////////////////////////////
//
//	libPCLFunclib.h:
//
//  Shuichi AKIZUKI
//
//	(C) 2015 ISL, Chukyo University All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef INCLUDE_libPCLFunclib_h_
#define INCLUDE_libPCLFunclib_h_
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include "akidata.h"
#include "aki3DVPM.h"
#include "libaki3DVPMlib.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "libHSVlib.h"

void RemoveFlatPoints( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr si, 
					   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cc,
					   float TH_FLAT,
					   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out );

void SavePointRGB( char ofname[], const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud );


void SamplingofVectorPairs( const pcl::PointCloud<pcl::PointNormal>::Ptr pnt,
							double3 mc, char *cm, double l1, double l2, double theta,
							double TH_L, double TH_THETA, 
							std::vector<vector_pair>& sampled );

void VPMarge( std::vector<vector_pair>& input, double th_dist, std::vector<vector_pair>& output );

void SamplingofVectorPairs2( const pcl::PointCloud<pcl::PointNormal>::Ptr pnt,
							double3 mc, char *cm, double l1, double l2, double theta,
							double TH_L, double TH_THETA, double marge_dist,
							std::vector<vector_pair>& sampled );

void FlatRemoving( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
				   float	radius, double threshold,
				   const pcl::PointCloud<pcl::PointNormal>::Ptr reduced );

// �_�Q�̉����DMat���|�C���^�n���ɂȂ��Ă��邱�Ƃɒ��ӁD�T�C�Y�͌Ăяo�����Œ�`���Ă������ƁD
void PCDNormal2CVMat( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud, double pitch, double3 trans, int fill, cv::Mat *img );

// �_�Q�̉����DMat���|�C���^�n���ɂȂ��Ă��邱�Ƃɒ��ӁD�T�C�Y�͌Ăяo�����Œ�`���Ă������ƁD
void PCDXYZ2CVMat( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double pitch, double3 trans, int fill, cv::Mat *img );

void PCD2CVMatScoreMap( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double pitch, double3 trans, int fill, cv::Mat scene, cv::Mat img );

void PCD2CVMatDrawCentroid( const double3 centroid, double pitch, double3 trans, cv::Mat scene, cv::Mat *dst );

void PCD2CVMatHypothesis( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,  const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_hyp,
							double pitch, double3 trans, cv::Mat img );

void CloudTransformPntNrm( const pcl::PointCloud<pcl::PointNormal>::Ptr src, double R[][3], double3 trans, const pcl::PointCloud<pcl::PointNormal>::Ptr dst );

// �_�Q�̔w�i�����������Ȃ��D
// ������̓_�Q�icloud_out�j�C�O�i�̃C���f�N�X�iIdx�j���o�͂����D
void CloudSubtraction( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bg, const double rad,
					  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::vector<int>& Idx );
void CloudSubtractionInv( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_bg, const double rad,
					  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::vector<int>& Idx );


// �_�Q�̃R�s�[�������Ȃ��D�ۑ�����ׂ�cloud_in�ɂ�����_��id��Idx�ɓ����Ă���D
void CopyPCD( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, pcl::PointCloud<int>& Idx,
				 pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out );
void CopyPCD( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<int>& Idx,
				 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out );
void CopyPCD( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<int>& Idx,
				 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out );

void CopyPCD( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, std::vector<int>& Idx,
				 pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out );

void CopyPCD( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, std::vector<int>& Idx,
				 pcl::PointCloud<pcl::PointNormal> *cloud_out );

void CopyPCD( const pcl::PointCloud<pcl::PointNormal> *cloud_in, std::vector<int>& Idx,
				 pcl::PointCloud<pcl::PointNormal> *cloud_out );

void CopyPCD( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::vector<int>& Idx,
				 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out );

void removeNaNNormalsFromPointCloud (const pcl::PointCloud<pcl::PointNormal> &cloud_in, 
									 pcl::PointCloud<pcl::PointNormal> &cloud_out,
                                     std::vector<int> &index);


void CalcCenter( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, pcl::PointXYZ *center );
void CalcCenter( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ *center );


void BoundaryExtractor( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, const double rate, const double th_curvature, std::vector<int>& Idx );

void Pnt2PCDXYZ( const double *pnt, const int n_points, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& Idx );
void Pnt2PCDXYZ( const double *pnt, cv::Mat img, const int n_points, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& Idx );
void Pnt2PCDXYZ_Edge( const double *pnt, cv::Mat img, const int n_points, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& Idx );


// �_�Q���ۑ��\�Ȃ̂����`�F�b�N����֐�
bool CheckSavePCD( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud );
bool CheckSavePCD( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud );
bool CheckSavePCD( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud );

// 20150425-Akizuki
// �r���̓����̓_�����o��
// dist_bin_front      �r���̑O�ʂ܂ł̋���
// camera_inclination  �J�����̌X��
// bin_width		   �r���̉���
void BinRemove( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double dist_bin_front, const double camera_inclination, const double bin_width,
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::vector<int>& Idx );

void BinRemove2( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double bin_width, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::vector<int>& Idx );

void BinAndNoiseRemove( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double bin_width, cv::Mat im_depth, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::vector<int>& Idx );


void PCDXYZ2CVMat_Mask( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double pitch, double3 trans, int fill, cv::Mat *img );

void SegmentationNoiseReduction( cv::Mat im_in, cv::Mat im_out );


#endif