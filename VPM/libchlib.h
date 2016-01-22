#ifndef INCLUDE_libchlib_h_
#define INCLUDE_libchlib_h_
#include "libHSVlib.h"

//特徴量の初期化と配列の生成
void InitializeCH( int level_, int max_, struct CH *colorHistogram ) ;
void InitializeCH2( int level_, int max_, struct CH *colorHistogram ) ;

//特徴量の読み込み
bool LoadCH( char name[], struct CH *ch ) ;

//特徴量（カラーヒストグラム）の算出
void CreateCH( unsigned char *color, int ie, int je, struct CH *colorHistogram ) ;

void CreateCHPCL( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, struct CH *colorHistogram ) ;

//モデル-シーン間の特徴量マッチング
void MatchingCH( const struct CH *ch_m, const struct CH *ch_s, double *score ) ;

//特徴量の保存
void SaveCH( char name[], struct CH *colorHistogram ) ;

//Debug
void DebugSave( char name[], unsigned char *color, int ie, int je ) ;

//配列の解放
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