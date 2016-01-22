#ifndef INCLUDE_ch_h_
#define INCLUDE_ch_h_

#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>


typedef struct CH {
	int		level, max ;
	int		*hist ;		// 実際のヒストグラム
	double	*dh ;		// 正規化ヒストグラム
} CH ;

typedef struct XYZ {
	double x, y, z ;
} XYZ ;

typedef struct SegPCD {
	pcl::PointCloud<pcl::PointXYZRGB> cloud ;
	std::vector<int>	id ;
	std::vector<bool>	flag ;
	std::vector<double>	outlinePro ;
	std::vector<XYZ>	centroid ;
	std::vector<std::vector<int>> segId ;
	int					id_max ;
} SegPCD ;

#endif