#ifndef INCLUDE_libchlib_h_
#define INCLUDE_libchlib_h_
#include "libHSVlib.h"

//�����ʂ̏������Ɣz��̐���
void InitializeCH( int level_, int max_, struct CH *colorHistogram ) ;
void InitializeCH2( int level_, int max_, struct CH *colorHistogram ) ;

//�����ʂ̓ǂݍ���
bool LoadCH( char name[], struct CH *ch ) ;

//�����ʁi�J���[�q�X�g�O�����j�̎Z�o
void CreateCH( unsigned char *color, int ie, int je, struct CH *colorHistogram ) ;

void CreateCHPCL( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, struct CH *colorHistogram ) ;

//���f��-�V�[���Ԃ̓����ʃ}�b�`���O
void MatchingCH( const struct CH *ch_m, const struct CH *ch_s, double *score ) ;

//�����ʂ̕ۑ�
void SaveCH( char name[], struct CH *colorHistogram ) ;

//Debug
void DebugSave( char name[], unsigned char *color, int ie, int je ) ;

//�z��̉��
void DeleteCH( struct CH *colorHistogram ) ;

void CopySegPCD( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, struct SegPCD *input_seg ) ;
void ComputeOutlineProbability( const double radius, struct SegPCD *input ) ;
double ComputeMeanOutlineProbability( struct SegPCD *input ) ;
void MergeRecursion( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int id, const double radius, const double c_th, pcl::KdTreeFLANN<pcl::PointXYZ> *kdtree, struct SegPCD *input_seg ) ;
void MergeSegByColor( const double radius, const double c_th, struct SegPCD *input ) ;
void ArrangeID( struct SegPCD *input_seg ) ;
void MergeSegByOutlineProbability( const int near_k, const int num_of_seg, struct SegPCD *input ) ;
void CopySegID( struct SegPCD *input, std::vector<std::vector<int>> &segIdx ) ;
void SegmentDomain( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, const int num_of_seg, const double radius, const double th_color, std::vector<std::vector<int>> &segIdx ) ;
void DebugOP( char name[], struct SegPCD *input ) ;
void DebugID( char name[], struct SegPCD *input ) ;

#endif