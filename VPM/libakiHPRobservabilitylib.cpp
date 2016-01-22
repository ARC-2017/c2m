//////////////////////////////////////////////////////////////////////////////
//
//	libakiHPRobservabilitylib.cpp:
//
//  Shuichi AKIZUKI
//
//	(C) 2015 ISL, Chukyo University All rights reserved.
//
//	This library consists of functions for calculating observability of
//	unorganized	point cloud data.
//	For calculating observability, we used the Hidden Point Removal operator
//	which is proposed in this paper.
//	Sagi Katz, Ayellet Tal, and Ronen Basri,"Direct Visibility of Point Sets", 
//	SIGGRAPH 2007.
//
//  Note:
//		2015.01.30
//			
//////////////////////////////////////////////////////////////////////////////
//#include "stdafx.h"
#include "libakiHPRobservabilitylib.h"



//void AkiCloudCentering( const pcl::PointCloud<pcl::PointNormal>::Ptr input,  pcl::PointXYZ *offset, const pcl::PointCloud<pcl::PointNormal>::Ptr output ){
//
//	output->width = input->width;
//	output->height = input->height;
//	pcl::PointNormal	ave;
//	ave.x = ave.y = ave.z = 0.0;
//	for( int i=0 ; i<input->points.size() ; i++ ){
//		ave.x += input->points[i].x;
//		ave.y += input->points[i].y;
//		ave.z += input->points[i].z;
//	}
//	ave.x /= (double)input->points.size();
//	ave.y /= (double)input->points.size();
//	ave.z /= (double)input->points.size();
//	for( int i=0 ; i<input->points.size() ; i++ ){
//		output->points.push_back( input->points[i] );
//		output->points[i].x - ave.x;
//		output->points[i].y - ave.y;
//		output->points[i].z - ave.z;
//	}
//
//	offset->x = ave.x;
//	offset->y = ave.y;
//	offset->z = ave.z;
//
//}
//
//void HPR_operator( const pcl::PointCloud<pcl::PointNormal>::Ptr input, pcl::PointXYZ view_point, double param, 
//				   pcl::PointCloud<int> &visibleIdx ){
//
//
//	pcl::PointXYZ	offset;
//	pcl::PointCloud<pcl::PointNormal>::Ptr c_input (new pcl::PointCloud<pcl::PointNormal>);
//	AkiCloudCentering( input, &offset, c_input );
//
//
//	pcl::PointCloud<pcl::PointNormal>::Ptr off_input (new pcl::PointCloud<pcl::PointNormal>);
//	pcl::copyPointCloud( *c_input, *off_input );
//
//	//fprintf( stderr,"view point( %5.2f, %5.2f, %5.2f )\n", view_point.x, view_point.y, view_point.z );
//
//#if DEBUG
//	double		size, tmp_size;
//	size = 0.0;
//	for( int i=0 ; i<off_input->points.size() ; i++ ){
//		tmp_size = sqrt( (off_input->points[i].x*off_input->points[i].x)
//			       + (off_input->points[i].y*off_input->points[i].y)
//				   + (off_input->points[i].z*off_input->points[i].z) );
//		if( size < tmp_size ) size = tmp_size;
//	}
//	fprintf( stderr,"object size = %5.2lf\n", size );
//#endif
//
//	//視点が原点になるようにオフセット
//	for( int i=0 ; i<off_input->points.size() ; i++ ){
//		off_input->points[i].x = off_input->points[i].x - view_point.x;
//		off_input->points[i].y = off_input->points[i].y - view_point.y;
//		off_input->points[i].z = off_input->points[i].z - view_point.z;
//	}
//	// ||p||の計算とその最大値の取得
//	std::vector<double> norm_p;
//	double				max_norm, norm;
//	norm_p.resize( off_input->points.size() );
//	max_norm = 0.0;
//	for( int i=0 ; i<off_input->points.size() ; i++ ){
//		norm = sqrt( (off_input->points[i].x*off_input->points[i].x)
//			       + (off_input->points[i].y*off_input->points[i].y)
//				   + (off_input->points[i].z*off_input->points[i].z) );
//		norm_p[i] = norm;
//		if( max_norm < norm ) max_norm = norm;
//	}
//#if DEBUG
//	fprintf( stderr,"Max norm = %5.2lf\n", max_norm );
//#endif
//
//	//Sphere radius
//	double radius;
//	radius = max_norm * pow( 10.0, param );
//
//	//Spherical flipping （視点データも追加する）
//	pcl::PointCloud<pcl::PointNormal>::Ptr flipped (new pcl::PointCloud<pcl::PointNormal>);
//	flipped->width = input->width + 1;
//	flipped->height = 1;
//	flipped->is_dense = false;
//	flipped->points.resize(flipped->width);
//	for( int i=0 ; i<flipped->points.size()-1 ; i++ ){
//		flipped->points[i].x = off_input->points[i].x + (2.0*(radius-norm_p[i])*(off_input->points[i].x/norm_p[i]));
//		flipped->points[i].y = off_input->points[i].y + (2.0*(radius-norm_p[i])*(off_input->points[i].y/norm_p[i]));
//		flipped->points[i].z = off_input->points[i].z + (2.0*(radius-norm_p[i])*(off_input->points[i].z/norm_p[i]));
//	}
//	flipped->points[ flipped->points.size()-1 ].x = view_point.x;
//	flipped->points[ flipped->points.size()-1 ].y = view_point.y;
//	flipped->points[ flipped->points.size()-1 ].z = view_point.z;
//
//	// construction of convex hull
//	pcl::PointCloud<pcl::PointNormal>::Ptr visible (new pcl::PointCloud<pcl::PointNormal>);
//	pcl::ConvexHull<pcl::PointNormal> convhull;
//	convhull.setDimension( 3 );
//	convhull.setInputCloud( flipped );
//	convhull.reconstruct( *visible );
//	//fprintf( stderr,"visible_point:%d\n", visible->points.size() );
//
//	// Indicesを使って高速化したいが，とりあえずkd-tree実装
//	pcl::KdTreeFLANN<pcl::PointNormal, flann::L2_Simple<float> > kdtree;
//	std::vector<int> pIdx;
//	std::vector<float> pD;
//	int					j;
//	kdtree.setInputCloud( flipped );
//	j = 0;
//	for( int i=0 ; i<visible->points.size() ; i++ ){
//		pIdx.clear();
//		pD.clear();
//		if( kdtree.nearestKSearch( visible->points[i], 1, pIdx, pD ) > 0 ){
//			if( (pD[0] == 0.0) && ( sqrt( (view_point.x-flipped->points[pIdx[0]].x)*(view_point.x-flipped->points[pIdx[0]].x)
//									    + (view_point.y-flipped->points[pIdx[0]].y)*(view_point.y-flipped->points[pIdx[0]].y)
//									    + (view_point.z-flipped->points[pIdx[0]].z)*(view_point.z-flipped->points[pIdx[0]].z) ) > 0.01 ) ){
//				visibleIdx.push_back( pIdx[0] );
//				//output->points[j] = input->points[ pIdx[0] ];
//				//fprintf( stderr,"No:%d dist: %lf id: %d\n", i, pD[0], pIdx[0] );
//				j++;
//			}
//		}
//	}
//
////	pcl::io::savePCDFileASCII( "flipped_cloud.pcd", *flipped );
////	pcl::io::savePCDFileASCII( "convhull.pcd", *visible );
//}


void AkiCloudCentering( const pcl::PointCloud<pcl::PointNormal>::Ptr input,  pcl::PointXYZ *offset, const pcl::PointCloud<pcl::PointNormal>::Ptr output ){

	output->width = input->width;
	output->height = input->height;
	pcl::PointNormal	ave;
	ave.x = ave.y = ave.z = 0.0;
	for( int i=0 ; i<input->points.size() ; i++ ){
		ave.x += input->points[i].x;
		ave.y += input->points[i].y;
		ave.z += input->points[i].z;
	}
	ave.x /= (double)input->points.size();
	ave.y /= (double)input->points.size();
	ave.z /= (double)input->points.size();
	for( int i=0 ; i<input->points.size() ; i++ ){
		output->points.push_back( input->points[i] );
		output->points[i].x -= ave.x;
		output->points[i].y -= ave.y;
		output->points[i].z -= ave.z;
	}

	offset->x = ave.x;
	offset->y = ave.y;
	offset->z = ave.z;

}

void AkiCloudCentering( const pcl::PointCloud<pcl::PointXYZ>::Ptr input,  pcl::PointXYZ *offset, const pcl::PointCloud<pcl::PointXYZ>::Ptr output ){

	output->width = input->width;
	output->height = input->height;
	pcl::PointNormal	ave;
	ave.x = ave.y = ave.z = 0.0;
	for( int i=0 ; i<input->points.size() ; i++ ){
		ave.x += input->points[i].x;
		ave.y += input->points[i].y;
		ave.z += input->points[i].z;
	}
	ave.x /= (double)input->points.size();
	ave.y /= (double)input->points.size();
	ave.z /= (double)input->points.size();
	for( int i=0 ; i<input->points.size() ; i++ ){
		output->points.push_back( input->points[i] );
		output->points[i].x -= ave.x;
		output->points[i].y -= ave.y;
		output->points[i].z -= ave.z;
	}

	offset->x = ave.x;
	offset->y = ave.y;
	offset->z = ave.z;

}

void HPR_operator( const pcl::PointCloud<pcl::PointNormal>::Ptr input, const double3 *view_point, double param, pcl::PointCloud<int> &visibleIdx ){


	pcl::PointXYZ	offset;
	pcl::PointCloud<pcl::PointNormal>::Ptr c_input (new pcl::PointCloud<pcl::PointNormal>);
	AkiCloudCentering( input, &offset, c_input );


	pcl::PointCloud<pcl::PointNormal>::Ptr off_input (new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud( *c_input, *off_input );

	//fprintf( stderr,"view point( %5.2f, %5.2f, %5.2f )\n", view_point.x, view_point.y, view_point.z );

#if DEBUG
	double		size, tmp_size;
	size = 0.0;
	for( int i=0 ; i<off_input->points.size() ; i++ ){
		tmp_size = sqrt( (off_input->points[i].x*off_input->points[i].x)
			       + (off_input->points[i].y*off_input->points[i].y)
				   + (off_input->points[i].z*off_input->points[i].z) );
		if( size < tmp_size ) size = tmp_size;
	}
	fprintf( stderr,"object size = %5.2lf\n", size );
#endif

	//視点が原点になるようにオフセット
	for( int i=0 ; i<off_input->points.size() ; i++ ){
		off_input->points[i].x = off_input->points[i].x - view_point->x;
		off_input->points[i].y = off_input->points[i].y - view_point->y;
		off_input->points[i].z = off_input->points[i].z - view_point->z;
	}
	// ||p||の計算とその最大値の取得
	std::vector<double> norm_p;
	double				max_norm, norm;
	norm_p.resize( off_input->points.size() );
	max_norm = 0.0;
	for( int i=0 ; i<off_input->points.size() ; i++ ){
		norm = sqrt( (off_input->points[i].x*off_input->points[i].x)
			       + (off_input->points[i].y*off_input->points[i].y)
				   + (off_input->points[i].z*off_input->points[i].z) );
		norm_p[i] = norm;
		if( max_norm < norm ) max_norm = norm;
	}
#if DEBUG
	fprintf( stderr,"Max norm = %5.2lf\n", max_norm );
#endif

	//Sphere radius
	double radius;
	radius = max_norm * pow( 10.0, param );

	//Spherical flipping （視点データも追加する）
	pcl::PointCloud<pcl::PointNormal>::Ptr flipped (new pcl::PointCloud<pcl::PointNormal>);
	flipped->width = input->width + 1;
	flipped->height = 1;
	flipped->is_dense = false;
	flipped->points.resize(flipped->width);
	for( int i=0 ; i<flipped->points.size()-1 ; i++ ){
		flipped->points[i].x = off_input->points[i].x + (2.0*(radius-norm_p[i])*(off_input->points[i].x/norm_p[i]));
		flipped->points[i].y = off_input->points[i].y + (2.0*(radius-norm_p[i])*(off_input->points[i].y/norm_p[i]));
		flipped->points[i].z = off_input->points[i].z + (2.0*(radius-norm_p[i])*(off_input->points[i].z/norm_p[i]));
	}
	flipped->points[ flipped->points.size()-1 ].x = view_point->x;
	flipped->points[ flipped->points.size()-1 ].y = view_point->y;
	flipped->points[ flipped->points.size()-1 ].z = view_point->z;

	// construction of convex hull
	pcl::PointCloud<pcl::PointNormal>::Ptr visible (new pcl::PointCloud<pcl::PointNormal>);
	pcl::ConvexHull<pcl::PointNormal> convhull;
	convhull.setDimension( 3 );
	convhull.setInputCloud( flipped );
	convhull.reconstruct( *visible );
	//fprintf( stderr,"visible_point:%d\n", visible->points.size() );

	// Indicesを使って高速化したいが，とりあえずkd-tree実装
	pcl::KdTreeFLANN<pcl::PointNormal, flann::L2_Simple<float> > kdtree;
	std::vector<int> pIdx;
	std::vector<float> pD;
	int					j;
	kdtree.setInputCloud( flipped );
	j = 0;
	for( int i=0 ; i<visible->points.size() ; i++ ){
		pIdx.clear();
		pD.clear();
		if( kdtree.nearestKSearch( visible->points[i], 1, pIdx, pD ) > 0 ){
			if( (pD[0] == 0.0) && ( sqrt( (view_point->x-flipped->points[pIdx[0]].x)*(view_point->x-flipped->points[pIdx[0]].x)
									    + (view_point->y-flipped->points[pIdx[0]].y)*(view_point->y-flipped->points[pIdx[0]].y)
									    + (view_point->z-flipped->points[pIdx[0]].z)*(view_point->z-flipped->points[pIdx[0]].z) ) > 0.01 ) ){
				visibleIdx.push_back( pIdx[0] );
				//output->points[j] = input->points[ pIdx[0] ];
				//fprintf( stderr,"No:%d dist: %lf id: %d\n", i, pD[0], pIdx[0] );
				j++;
			}
		}
	}

//	pcl::io::savePCDFileASCII( "flipped_cloud.pcd", *flipped );
//	pcl::io::savePCDFileASCII( "convhull.pcd", *visible );
}

void HPR_operator( const pcl::PointCloud<pcl::PointXYZ>::Ptr input, const double3 *view_point, double param, pcl::PointCloud<int> &visibleIdx ){


	pcl::PointXYZ	offset;
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_input (new pcl::PointCloud<pcl::PointXYZ>);
	AkiCloudCentering( input, &offset, c_input );


	pcl::PointCloud<pcl::PointXYZ>::Ptr off_input (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud( *c_input, *off_input );

	//fprintf( stderr,"view point( %5.2f, %5.2f, %5.2f )\n", view_point.x, view_point.y, view_point.z );

#if DEBUG
	double		size, tmp_size;
	size = 0.0;
	for( int i=0 ; i<off_input->points.size() ; i++ ){
		tmp_size = sqrt( (off_input->points[i].x*off_input->points[i].x)
			       + (off_input->points[i].y*off_input->points[i].y)
				   + (off_input->points[i].z*off_input->points[i].z) );
		if( size < tmp_size ) size = tmp_size;
	}
	fprintf( stderr,"object size = %5.2lf\n", size );
#endif

	//視点が原点になるようにオフセット
	for( int i=0 ; i<off_input->points.size() ; i++ ){
		off_input->points[i].x = off_input->points[i].x - view_point->x;
		off_input->points[i].y = off_input->points[i].y - view_point->y;
		off_input->points[i].z = off_input->points[i].z - view_point->z;
	}
	// ||p||の計算とその最大値の取得
	std::vector<double> norm_p;
	double				max_norm, norm;
	norm_p.resize( off_input->points.size() );
	max_norm = 0.0;
	for( int i=0 ; i<off_input->points.size() ; i++ ){
		norm = sqrt( (off_input->points[i].x*off_input->points[i].x)
			       + (off_input->points[i].y*off_input->points[i].y)
				   + (off_input->points[i].z*off_input->points[i].z) );
		norm_p[i] = norm;
		if( max_norm < norm ) max_norm = norm;
	}
#if DEBUG
	fprintf( stderr,"Max norm = %5.2lf\n", max_norm );
#endif

	//Sphere radius
	double radius;
	radius = max_norm * pow( 10.0, param );

	//Spherical flipping （視点データも追加する）
	pcl::PointCloud<pcl::PointXYZ>::Ptr flipped (new pcl::PointCloud<pcl::PointXYZ>);
	flipped->width = input->width + 1;
	flipped->height = 1;
	flipped->is_dense = false;
	flipped->points.resize(flipped->width);
	for( int i=0 ; i<flipped->points.size()-1 ; i++ ){
		flipped->points[i].x = off_input->points[i].x + (2.0*(radius-norm_p[i])*(off_input->points[i].x/norm_p[i]));
		flipped->points[i].y = off_input->points[i].y + (2.0*(radius-norm_p[i])*(off_input->points[i].y/norm_p[i]));
		flipped->points[i].z = off_input->points[i].z + (2.0*(radius-norm_p[i])*(off_input->points[i].z/norm_p[i]));
	}
	flipped->points[ flipped->points.size()-1 ].x = view_point->x;
	flipped->points[ flipped->points.size()-1 ].y = view_point->y;
	flipped->points[ flipped->points.size()-1 ].z = view_point->z;

	// construction of convex hull
	pcl::PointCloud<pcl::PointXYZ>::Ptr visible (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> convhull;
	convhull.setDimension( 3 );
	convhull.setInputCloud( flipped );
	convhull.reconstruct( *visible );
	//fprintf( stderr,"visible_point:%d\n", visible->points.size() );

	// Indicesを使って高速化したいが，とりあえずkd-tree実装
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float> > kdtree;
	std::vector<int> pIdx;
	std::vector<float> pD;
	int					j;
	kdtree.setInputCloud( flipped );
	j = 0;
	for( int i=0 ; i<visible->points.size() ; i++ ){
		pIdx.clear();
		pD.clear();
		if( kdtree.nearestKSearch( visible->points[i], 1, pIdx, pD ) > 0 ){
			if( (pD[0] == 0.0) && ( sqrt( (view_point->x-flipped->points[pIdx[0]].x)*(view_point->x-flipped->points[pIdx[0]].x)
									    + (view_point->y-flipped->points[pIdx[0]].y)*(view_point->y-flipped->points[pIdx[0]].y)
									    + (view_point->z-flipped->points[pIdx[0]].z)*(view_point->z-flipped->points[pIdx[0]].z) ) > 0.01 ) ){
				visibleIdx.push_back( pIdx[0] );
				//output->points[j] = input->points[ pIdx[0] ];
				//fprintf( stderr,"No:%d dist: %lf id: %d\n", i, pD[0], pIdx[0] );
				j++;
			}
		}
	}

//	pcl::io::savePCDFileASCII( "flipped_cloud.pcd", *flipped );
//	pcl::io::savePCDFileASCII( "convhull.pcd", *visible );
}

void HPR_operator( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputXYZRGB, const double3 *view_point, double param, std::vector<int> &visibleIdx ){

	// PointXYZRGB -> PointXYZ	
	pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
	input->width = inputXYZRGB->width;
	input->height = inputXYZRGB->height;
	input->points.resize( inputXYZRGB->points.size() );
	for( int i=0 ; i<input->points.size() ; i++ ){
		input->points[i].x = inputXYZRGB->points[i].x;
		input->points[i].y = inputXYZRGB->points[i].y;
		input->points[i].z = inputXYZRGB->points[i].z;
	}

	pcl::PointXYZ	offset;
	pcl::PointCloud<pcl::PointXYZ>::Ptr c_input (new pcl::PointCloud<pcl::PointXYZ>);
	AkiCloudCentering( input, &offset, c_input );


	pcl::PointCloud<pcl::PointXYZ>::Ptr off_input (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud( *c_input, *off_input );

	//fprintf( stderr,"view point( %5.2f, %5.2f, %5.2f )\n", view_point.x, view_point.y, view_point.z );

#if DEBUG
	double		size, tmp_size;
	size = 0.0;
	for( int i=0 ; i<off_input->points.size() ; i++ ){
		tmp_size = sqrt( (off_input->points[i].x*off_input->points[i].x)
			       + (off_input->points[i].y*off_input->points[i].y)
				   + (off_input->points[i].z*off_input->points[i].z) );
		if( size < tmp_size ) size = tmp_size;
	}
	fprintf( stderr,"object size = %5.2lf\n", size );
#endif

	//視点が原点になるようにオフセット
	for( int i=0 ; i<off_input->points.size() ; i++ ){
		off_input->points[i].x = off_input->points[i].x - view_point->x;
		off_input->points[i].y = off_input->points[i].y - view_point->y;
		off_input->points[i].z = off_input->points[i].z - view_point->z;
	}
	// ||p||の計算とその最大値の取得
	std::vector<double> norm_p;
	double				max_norm, norm;
	norm_p.resize( off_input->points.size() );
	max_norm = 0.0;
	for( int i=0 ; i<off_input->points.size() ; i++ ){
		norm = sqrt( (off_input->points[i].x*off_input->points[i].x)
			       + (off_input->points[i].y*off_input->points[i].y)
				   + (off_input->points[i].z*off_input->points[i].z) );
		norm_p[i] = norm;
		if( max_norm < norm ) max_norm = norm;
	}
#if DEBUG
	fprintf( stderr,"Max norm = %5.2lf\n", max_norm );
#endif

	//Sphere radius
	double radius;
	radius = max_norm * pow( 10.0, param );

	//Spherical flipping （視点データも追加する）
	pcl::PointCloud<pcl::PointXYZ>::Ptr flipped (new pcl::PointCloud<pcl::PointXYZ>);
	flipped->width = input->width + 1;
	flipped->height = 1;
	flipped->is_dense = false;
	flipped->points.resize(flipped->width);
	for( int i=0 ; i<flipped->points.size()-1 ; i++ ){
		flipped->points[i].x = off_input->points[i].x + (2.0*(radius-norm_p[i])*(off_input->points[i].x/norm_p[i]));
		flipped->points[i].y = off_input->points[i].y + (2.0*(radius-norm_p[i])*(off_input->points[i].y/norm_p[i]));
		flipped->points[i].z = off_input->points[i].z + (2.0*(radius-norm_p[i])*(off_input->points[i].z/norm_p[i]));
	}
	flipped->points[ flipped->points.size()-1 ].x = view_point->x;
	flipped->points[ flipped->points.size()-1 ].y = view_point->y;
	flipped->points[ flipped->points.size()-1 ].z = view_point->z;

	// construction of convex hull
	pcl::PointCloud<pcl::PointXYZ>::Ptr visible (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> convhull;
	convhull.setDimension( 3 );
	convhull.setInputCloud( flipped );
	convhull.reconstruct( *visible );
	//fprintf( stderr,"visible_point:%d\n", visible->points.size() );

	// Indicesを使って高速化したいが，とりあえずkd-tree実装
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float> > kdtree;
	std::vector<int> pIdx;
	std::vector<float> pD;
	int					j;
	kdtree.setInputCloud( flipped );
	j = 0;
	for( int i=0 ; i<visible->points.size() ; i++ ){
		pIdx.clear();
		pD.clear();
		if( kdtree.nearestKSearch( visible->points[i], 1, pIdx, pD ) > 0 ){
			if( (pD[0] == 0.0) && ( sqrt( (view_point->x-flipped->points[pIdx[0]].x)*(view_point->x-flipped->points[pIdx[0]].x)
									    + (view_point->y-flipped->points[pIdx[0]].y)*(view_point->y-flipped->points[pIdx[0]].y)
									    + (view_point->z-flipped->points[pIdx[0]].z)*(view_point->z-flipped->points[pIdx[0]].z) ) > 0.01 ) ){
				visibleIdx.push_back( pIdx[0] );
				//output->points[j] = input->points[ pIdx[0] ];
				//fprintf( stderr,"No:%d dist: %lf id: %d\n", i, pD[0], pIdx[0] );
				j++;
			}
		}
	}

//	pcl::io::savePCDFileASCII( "flipped_cloud.pcd", *flipped );
//	pcl::io::savePCDFileASCII( "convhull.pcd", *visible );
}



void AkiCreateObservabilityMap( const int nObserve, const int nPoints, struct observability_map *om ){

	om->nObserve = nObserve;
	om->nPoints = nPoints;
	//Observabilityの領域確保
	om->obs.resize( nPoints );
	om->view_point.resize( nObserve );
	for( int i=0 ; i<nPoints ; i++ ){
		om->obs[i].resize( nObserve, 0 );
	}
	//om->obs = (char **)malloc( sizeof(char *)*nPoints );
	//om->view_point = (double3 *)malloc( sizeof(double3)*nObserve );
	//for( int i=0 ; i<nPoints ; i++ ){
	//	om->obs[i] = (char *)malloc( sizeof(char)*nObserve );
	//}
	//for( int j=0 ; j<nPoints ; j++ ){
	//	for( int i=0 ; i<nObserve ; i++ ){
	//		om->obs[j][i] = 0;
	//	}
	//}
	for( int i=0 ; i<nObserve ; i++ ){
		om->view_point[i].x = 0.0;
		om->view_point[i].y = 0.0;
		om->view_point[i].z = 0.0;
	}
}

void AkiSaveObservabilityMapPCD( const pcl::PointCloud<pcl::PointNormal>::Ptr pnt, const observability_map *om, 
								 const char obsmappcdfname[] ){

	int		nPoints, nObserve, cnt;

	nPoints = pnt->points.size();
	nObserve = om->nObserve;
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr obs_map (new pcl::PointCloud<pcl::PointXYZHSV>);
	obs_map->width = pnt->points.size();
	obs_map->height = 1;
	obs_map->points.resize( obs_map->width );
	for( int i=0 ; i<nPoints ; i++ ){
		cnt = 0;
		for( int j=0 ; j<nObserve ; j++ ){
			cnt += om->obs[i][j];
		}
		obs_map->points[i].x = pnt->points[i].x;
		obs_map->points[i].y = pnt->points[i].y;
		obs_map->points[i].z = pnt->points[i].z;
		obs_map->points[i].h = 0.6*(1.0 - (float)cnt/(float)nObserve);
		obs_map->points[i].s = 1.0;
		obs_map->points[i].v = 1.0;

	}
#if OUTPUT2
	pcl::io::savePCDFileASCII( obsmappcdfname, *obs_map );
#endif
}

void AkiCalcObservabilityOfVectorPairs( std::vector<vector_pair>& vp, struct observability_map *om ){

	int		nVP, nObserve;
	int		obs;

	nVP = vp.size();
	nObserve = om->nObserve;

	for( int j=0 ; j<nVP ; j++ ){
		obs = 0;
		for( int i=0 ; i<nObserve ; i++ ){
			obs += om->obs[ vp[j].p_idx ][i] * om->obs[ vp[j].q1_idx ][i] * om->obs[ vp[j].q2_idx ][i];
		}
		vp[j].observability = (double)( (double)obs/(double)nObserve );
	}
}

