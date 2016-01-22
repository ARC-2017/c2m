//#include "stdafx.h"

#include "common2.h"
#include "ch.h"
#include "libchlib.h"

//OpenCV
//#include <opencv/cv.h>
//#include <opencv/highgui.h>

void InitializeCH( int level_, int max_, struct CH *colorHistogram ) {

	colorHistogram->level = level_ ;
	colorHistogram->max = max_ ;
	colorHistogram->hist = (int *)malloc( sizeof(int)*(level_*3) ) ;
	colorHistogram->dh   = (double *)malloc( sizeof(double)*(level_*3) ) ;
	for( int i=0, l=level_*3 ; i<l ; i++ ) {
		colorHistogram->hist[i] = 0 ;
		colorHistogram->dh[i] = 0.0 ;
	}
	return ;
}

void InitializeCH2( int level_, int max_, struct CH *colorHistogram ) {

	colorHistogram->level = level_ ;
	colorHistogram->max = max_ ;
	colorHistogram->hist = (int *)malloc( sizeof(int)*(level_) ) ;
	colorHistogram->dh   = (double *)malloc( sizeof(double)*(level_) ) ;
	for( int i=0, l=level_*3 ; i<l ; i++ ) {
		colorHistogram->hist[i] = 0 ;
		colorHistogram->dh[i] = 0.0 ;
	}
	return ;
}

bool LoadCH( char name[], struct CH *ch ) {

	FILE		*fp ;
	char		dum[256];
	errno_t		error_check;
	if( (error_check = fopen_s( &fp, name, "r" )) != 0 ) {
		fprintf( stderr, "!!Error %s cannot be load.\n", name ) ;
		return false ;
	}
	if( fscanf( fp, "%s %d", dum, &(ch->level) ) < 2 ) {
		fprintf( stderr,"!!Error in LoadCH.\n" ) ;
		return false ;
	}
	if( fscanf( fp, "%s %d", dum, &(ch->max) ) < 2 ) {
		fprintf( stderr,"!!Error in LoadCH.\n" ) ;
		return false ;
	}
	ch->hist = (int *)malloc( sizeof(int)*((ch->level)*3) ) ;
	ch->dh = (double *)malloc( sizeof(double)*((ch->level)*3) ) ;
	for( int i=0, l=(ch->level)*3 ; i<l ; i++ ) {
		if( fscanf( fp, "%lf", &(ch->dh[i]) ) < 1 ) {
			fprintf( stderr,"!!Error in LoadCH.\n" ) ;
			return false ;
		}
	}
	fclose( fp ) ;
	return true ;
}

void CreateCH( unsigned char *color, int ie, int je, struct CH *colorHistogram ) {

	int		level, level2 ;
	int		id_r, id_g, id_b ;
	int		max_i ;
	int		i_sum ;
	double	base ;
	unsigned char r, g, b ;
	level  = colorHistogram->level ;
	level2 = level*2 ;
	max_i = colorHistogram->max ;
	base  = (double)max_i/(double)(level-1) ;
	i_sum = 0 ;
	for( int j=0 ; j<je ; j++ ) {
		for( int i=0 ; i<ie ; i++ ) {
			r = color[(j*ie*3)+(i*3)] ;
			g = color[(j*ie*3)+(i*3)+1] ;
			b = color[(j*ie*3)+(i*3)+2] ;
			if( !r && !g &&  !b ) continue ;
			id_r = (int)((double)r/base) ;
			id_g = (int)((double)g/base) + level  ;
			id_b = (int)((double)b/base) + level2 ;
			colorHistogram->hist[id_r]++ ;
			colorHistogram->hist[id_g]++ ;
			colorHistogram->hist[id_b]++ ;
			i_sum += 3 ;
		}
	}
	//合計1に正規化
	for( int i=0, l=level*3 ; i<l ; i++ ) {
		colorHistogram->dh[i] = (double)colorHistogram->hist[i]/(double)i_sum ;
	}
	return ;

}

void CreateCHPCL( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, struct CH *colorHistogram ) {

	int		num, level, level2 ;
	int		id_r, id_g, id_b ;
	int		max_i ;
	double	base ;
	unsigned char r, g, b ;
	level  = colorHistogram->level ;
	level2 = level*2 ;
	max_i = colorHistogram->max ;
	base  = (double)max_i/(double)(level-1) ;
	num = cloud->points.size() ;
	for( int i=0 ; i<num ; i++ ) {
		r = cloud->points[i].r ;
		g = cloud->points[i].g ;
		b = cloud->points[i].b ;
		id_r = (int)((double)r/base) ;
		id_g = (int)((double)g/base) + level  ;
		id_b = (int)((double)b/base) + level2 ;
		colorHistogram->hist[id_r]++ ;
		colorHistogram->hist[id_g]++ ;
		colorHistogram->hist[id_b]++ ;
	}
	//合計1に正規化
	for( int i=0, l=level*3 ; i<l ; i++ ) {
		colorHistogram->dh[i] = (double)colorHistogram->hist[i]/(double)(num*3) ;
	}
	return ;

}

void MatchingCH( const struct CH *ch_m, const struct CH *ch_s, double *score ) {

	double tmp_score ;
	tmp_score = 0.0 ;
	//KL divergence
	for( int i=0, l=(ch_m->level)*3 ; i<l ; i++ ) {
		if( ch_m->dh[i] && ch_s->dh[i] ) tmp_score += (ch_m->dh[i]-ch_s->dh[i]) * log( ch_m->dh[i]/ch_s->dh[i] ) ;
	}
	*score = tmp_score ;
	return ;

}


void SaveCH( char name[], struct CH *colorHistogram ) {

	FILE	*fp;
	fp = fopen( name, "w" );
	fprintf( fp,"level: %d\n", colorHistogram->level );
	fprintf( fp,"max_intensity: %d\n", colorHistogram->max );
	for( int i=0, l=(colorHistogram->level)*3 ; i<l ; i++ ) {
		fprintf( fp, "%lf\n", colorHistogram->dh[i] );
	}
	fclose( fp ) ;
	return ;

}

void DebugSave( char name[], unsigned char *color, int ie, int je ) {

	cv::Mat	img_ ;
	img_ = cv::Mat::zeros( je, ie, CV_8UC3 );
	for( int j=0 ; j<je ; j++ ){
		for( int i=0 ; i<ie ; i++ ){
			cv::Vec3d tmp ;
			tmp(0) = color[(j*ie*3)+(i*3)] ;
			tmp(1) = color[(j*ie*3)+(i*3)+1] ;
			tmp(2) = color[(j*ie*3)+(i*3)+2] ;
			img_.at<cv::Vec3b>( j, i ) = tmp ;
		}
	}
	cv::imwrite( name, img_ );
	return ;
}

void DeleteCH( struct CH *colorHistogram ) {

	free( colorHistogram->hist ) ;
	free( colorHistogram->dh   ) ;
	return ;

}
//New segmentation algorithm

void CopySegPCD( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, struct SegPCD *input_seg ) {

	int	num = input->points.size() ;
	input_seg->cloud.height = num ;	input_seg->cloud.width = 1 ;	input_seg->cloud.is_dense = false ;
	input_seg->cloud.points.resize( num ) ;
	for( int i=0 ; i<num ; i++ ) {
		input_seg->cloud.points[i].x = input->points[i].x ;
		input_seg->cloud.points[i].y = input->points[i].y ;
		input_seg->cloud.points[i].z = input->points[i].z ;
		input_seg->cloud.points[i].r = input->points[i].r ;
		input_seg->cloud.points[i].g = input->points[i].g ;
		input_seg->cloud.points[i].b = input->points[i].b ;
	}
	return ;

}

void ComputeOutlineProbability( const double radius, struct SegPCD *input ) {

	int		n, num ;
	double	max_ ;
	num = input->cloud.points.size() ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> ) ;
	cloud->height = num ; cloud->width = 1 ;	cloud->is_dense = false ;
	cloud->points.resize( num ) ;
	for( int i=0 ; i<num ; i++ ) {
		cloud->points[i].x = input->cloud.points[i].x ;
		cloud->points[i].y = input->cloud.points[i].y ;
		cloud->points[i].z = input->cloud.points[i].z ;
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree ;
	std::vector<int>	pointID( num ) ;
	std::vector<float>	pointSD( num ) ;
	kdtree.setInputCloud( cloud ) ;
	pcl::PointXYZ point ;
	max_ = 0.0 ;
	for( int i=0 ; i<num ; i++ ) {
		point.x = cloud->points[i].x ;
		point.y = cloud->points[i].y ;
		point.z = cloud->points[i].z ;
		n = kdtree.radiusSearch( point, radius, pointID, pointSD ) ;
		double	tmp=0.0 ;
		for( int k=0 ; k<n ; k++ ) tmp += (radius-sqrt( pointSD[k] ) ) ;
		if( max_<tmp ) max_ = tmp ;
		input->outlinePro.push_back( tmp ) ;
	}
	for( int i=0 ; i<num ; i++ ) {
		if( 0.0<max_ )	input->outlinePro[i] /= max_ ;
		else			input->outlinePro[i] = 0.0 ;
	}
	return ;

}

double ComputeMeanOutlineProbability( struct SegPCD *input ) {

	int		num ;
	double	meanOP ;
	num = input->outlinePro.size() ;
	meanOP = 0.0 ;
	for( int i=0 ; i<num ; i++ ) meanOP += input->outlinePro[i] ;
	meanOP /= (double)num ;
	return meanOP ;

}

void MergeRecursion( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int id, const double radius, const double c_th, pcl::KdTreeFLANN<pcl::PointXYZ> *kdtree, struct SegPCD *input_seg ) {

	int		n, num ;
	double	nor, c_th2 ;
	//uint8_t r1, g1, b1, r2, g2, b2 ;
	unsigned char r1, g1, b1, r2, g2, b2 ;
	num = cloud->points.size() ;
	std::vector<int>	pointID( num ) ;
	std::vector<float>	pointSD( num ) ;
	n = kdtree->radiusSearch( cloud->points[id], radius, pointID, pointSD ) ;
	c_th2 = c_th*c_th ;
	for( int i=0 ; i<n ; i++ ) {
		r1 = input_seg->cloud.points[id].r ;			g1 = input_seg->cloud.points[id].g ;			b1 = input_seg->cloud.points[id].b ;
		r2 = input_seg->cloud.points[pointID[i]].r ;	g2 = input_seg->cloud.points[pointID[i]].g ;	b2 = input_seg->cloud.points[pointID[i]].b ;
		nor = (r1-r2)*(r1-r2) + (g1-g2)*(g1-g2) + (b1-b2)*(b1-b2) ;
		if( (nor<c_th2) && !(input_seg->flag[pointID[i]]) ) {
			input_seg->id[pointID[i]] = input_seg->id[id] ;
			input_seg->flag[pointID[i]] = true ;
			MergeRecursion( cloud, pointID[i], radius, c_th, kdtree, input_seg ) ;
		}
	}
	return ;

}

void MergeSegByColor( const double radius, const double c_th, struct SegPCD *input ) {

	int	num ;
	num = input->cloud.points.size() ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> ) ;
	cloud->height = num ; cloud->width = 1 ;	cloud->is_dense = false ;
	cloud->points.resize( num ) ;
	for( int i=0 ; i<num ; i++ ) {
		cloud->points[i].x = input->cloud.points[i].x ;
		cloud->points[i].y = input->cloud.points[i].y ;
		cloud->points[i].z = input->cloud.points[i].z ;
		input->id.push_back( i ) ;
		input->flag.push_back( false ) ;
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree ;
	kdtree.setInputCloud( cloud ) ;
	for( int i=0 ; i<num ; i++ ) {
		if( input->flag[i] ) continue ;
		input->flag[i] = true ;
		MergeRecursion( cloud, i, radius, c_th, &kdtree, input ) ;
	}
	return ;

}

//IDを0～Nに整理（元は，クラス番号が割り振られているだけで0から順に並んでいるわけではない）
void ArrangeID( struct SegPCD *input_seg ) {

	int		num, id ;
	bool	tf ;
	std::vector<int> a_id ; // arrangement ID 
	num = input_seg->cloud.points.size() ;
	for( int i=0 ; i<num ; i++ ) {
		tf = true ;
		id = input_seg->id[i] ;
		for( int n=0, l=a_id.size() ; n<l ; n++ ) {
			if( a_id[n]==id )	tf = false ;
		}
		if( tf ) a_id.push_back( id ) ;
	}
	for( int i=0 ; i<num ; i++ ) {
		id = input_seg->id[i] ;
		for( int n=0, l=a_id.size() ; n<l ; n++ ) {
			if( a_id[n]==id ) {
				input_seg->id[i] = n ;
				break ;
			}
		}
	}
	input_seg->id_max = a_id.size() ;
	return ;
}

//物体領域の確率値をもとにセグメントをマージ
void MergeSegByOutlineProbability( const int near_k, const int num_of_seg, struct SegPCD *input ) {

	int				num, id, id1, id2, num_seg, id_max, id_max_tmp, id_seg, num_k ;
	double			pro_th, shiftVal ;
	XYZ				mean ;
	num = input->cloud.points.size() ;
	pro_th = ComputeMeanOutlineProbability( input ) ;	//確率の平均値を計算
	shiftVal = pro_th/2.0 ;
	pro_th += shiftVal ;
	//指定したセグメント数になるまで繰り返し
	while( num_of_seg<input->id_max ) {
		id_max = input->id_max ;
		input->segId.clear( ) ;
		//セグメントごとにもとの点群のIDを保存
		input->segId.resize( id_max ) ;
		for( int i=0 ; i<num ; i++ ) {
			input->segId[input->id[i]].push_back( i ) ;
		}
		//各セグメントの重心位置を算出
		pcl::PointCloud<pcl::PointXYZ>::Ptr centroids( new pcl::PointCloud<pcl::PointXYZ> ) ;
		centroids->height = id_max ; centroids->width = 1 ;	centroids->is_dense = false ;
		centroids->points.resize( id_max ) ;
		input->flag.clear( ) ;
		input->centroid.clear( ) ;
		for( int i=0; i<id_max ; i++ ) {
			input->flag.push_back( false ) ;	//フラグを初期化
			mean.x = mean.y = mean.z = 0.0 ;
			num_seg = input->segId[i].size();
			for( int n=0 ; n<num_seg ; n++ ) {
				id = input->segId[i][n] ;
				mean.x += input->cloud.points[id].x ;
				mean.y += input->cloud.points[id].y ;
				mean.z += input->cloud.points[id].z ;
			}
			mean.x /= (double)num_seg ;	mean.y /= (double)num_seg ;	mean.z /= (double)num_seg ;
			input->centroid.push_back( mean ) ;
			centroids->points[i].x = mean.x ;	centroids->points[i].y = mean.y ;	centroids->points[i].z = mean.z ;
		}
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_centroid ;
		kdtree_centroid.setInputCloud( centroids ) ;
		std::vector<int>	pointID_centroid( 2 ) ;
		std::vector<float>	pointSD_centroid( 2 ) ;
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_ ;
		std::vector<int>	pointID_( near_k ) ;
		std::vector<float>	pointSD_( near_k ) ;
		id_max_tmp = id_max ;
		for( int i=0; i<id_max ; i++ ) {
			if( input->flag[i] ) continue ;	//フラグが立っているときはスキップ
			//近傍のセグメントを検出
			kdtree_centroid.nearestKSearch( centroids->points[i], 2, pointID_centroid, pointSD_centroid ) ;
			//ID取得 + セグメントごとの Point Cloud 生成
			id1 = i ;	id2 = pointID_centroid[1] ;
			if( input->flag[id2] ) continue ;	//相手側のセグメントのフラグが立っているときもスキップ
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_id1( new pcl::PointCloud<pcl::PointXYZ> ) ;
			cloud_id1->height = input->segId[id1].size() ; cloud_id1->width = 1 ;	cloud_id1->is_dense = false ;
			cloud_id1->points.resize( cloud_id1->height*cloud_id1->width ) ;
			for( int n=0, m=cloud_id1->points.size() ; n<m ; n++ ) {
				id_seg = input->segId[id1][n] ;
				cloud_id1->points[n].x = input->cloud.points[id_seg].x ;
				cloud_id1->points[n].y = input->cloud.points[id_seg].y ;
				cloud_id1->points[n].z = input->cloud.points[id_seg].z ;
			}
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_id2( new pcl::PointCloud<pcl::PointXYZ> ) ;
			cloud_id2->height = input->segId[id2].size() ; cloud_id2->width = 1 ;	cloud_id2->is_dense = false ;
			cloud_id2->points.resize( cloud_id2->height*cloud_id2->width ) ;
			for( int n=0, m=cloud_id2->points.size() ; n<m ; n++ ) {
				id_seg = input->segId[id2][n] ;
				cloud_id2->points[n].x = input->cloud.points[id_seg].x ;
				cloud_id2->points[n].y = input->cloud.points[id_seg].y ;
				cloud_id2->points[n].z = input->cloud.points[id_seg].z ;
			}
			//
			bool c_t=false ; //flagの生成 2015.05.27 武井
			std::vector<double> outLpro ;
			kdtree_.setInputCloud( cloud_id1 ) ;
			num_k = kdtree_.nearestKSearch( centroids->points[id2], near_k, pointID_, pointSD_ ) ;
			if( num_k<near_k ) c_t=true ; //点数が少ないかチェック 2015.05.27 武井
			for( int n=0 ; n<num_k ; n++ ) {
				outLpro.push_back( input->outlinePro[input->segId[id1][pointID_[n]]] ) ;
			}
			kdtree_.setInputCloud( cloud_id2 ) ;
			num_k = kdtree_.nearestKSearch( centroids->points[id1], near_k, pointID_, pointSD_ ) ;
			if( num_k<near_k ) c_t=true ; //点数が少ないかチェック 2015.05.27 武井
			for( int n=0 ; n<num_k ; n++ ) {
				outLpro.push_back( input->outlinePro[input->segId[id2][pointID_[n]]] ) ;
			}
			//相手側のセグメントに近い点群が持つ確率がしきい値よりも高い場合，マージ
			int	k ;
			k = 0 ;
			for( int n=0, l=outLpro.size() ; n<l ; n++ ) {
				if( pro_th<outLpro[n] )	k++ ;
			}
			if( k==outLpro.size() || c_t ) { //点数が少ない場合に統合する条件式を追加 2015.05.27 武井
			//if( k==outLpro.size() ) {
				for( int n=0, m=input->segId[id2].size() ; n<m ; n++ ) {
					id_seg = input->segId[id2][n] ;
					input->id[id_seg] = id1 ;
				}
				id_max_tmp-- ;	//マージしたときはセグメント数を減らす
				if( id_max_tmp<=num_of_seg ) break ;	//指定したセグメント数なら終了
				input->flag[id1] = input->flag[id2] = true ;	//マージしたときはフラグを立てる
			}
		}
		ArrangeID( input ) ;
		pro_th -= shiftVal*0.01 ; //更新を低速化 2015.05.27 武井
	}
	input->segId.clear( ) ;
	//セグメントごとにもとの点群のIDを保存
	input->segId.resize( id_max ) ;
	for( int i=0 ; i<num ; i++ ) {
		input->segId[input->id[i]].push_back( i ) ;
	}
	return ;

}

void CopySegID( struct SegPCD *input, std::vector<std::vector<int>> &segIdx ) {

	int num_id ;
	num_id = input->id_max ;
	segIdx.resize( num_id ) ;
	for( int i=0 ; i<num_id ; i++ ) {
		for( int n=0, l=input->segId[i].size() ; n<l ; n++ ) {
			segIdx[i].push_back( input->segId[i][n] ) ;
		}
	}
	return ;

}

void SegmentDomain( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, const int num_of_seg, const double radius, const double th_color, std::vector<std::vector<int>> &segIdx ) {

	SegPCD input_seg ;
	CopySegPCD( input, &input_seg ) ;
	//fprintf( stderr, "OK!\n" ) ;
	ComputeOutlineProbability( radius, &input_seg ) ;
	//fprintf( stderr, "OK!\n" ) ;
	MergeSegByColor( radius, th_color, &input_seg ) ;
	ArrangeID( &input_seg ) ;
	//DebugID( "debugID_color.pcd", &input_seg ) ;
	MergeSegByOutlineProbability( 10, num_of_seg, &input_seg ) ;
	CopySegID( &input_seg, segIdx ) ;
	//DebugOP( "outlinePro.pcd", &input_seg ) ;
	//DebugID( "debugID.pcd", &input_seg ) ;
	return ;

}

void DebugOP( char name[], struct SegPCD *input ) {

	int num ;
	num = input->cloud.points.size() ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr color( new pcl::PointCloud<pcl::PointXYZRGB> ) ;
	color->height = num ;	color->width = 1 ;	color->is_dense = false ;
	color->points.resize( num ) ;
	//uint8_t r, g, b ;
	unsigned char r, g, b ;
	uint32_t rgb ;
	for( int i=0 ; i<num ; i++ ) {
		color->points[i].x = input->cloud.points[i].x ;
		color->points[i].y = input->cloud.points[i].y ;
		color->points[i].z = input->cloud.points[i].z ;
		double	dn_in = input->outlinePro[i] * 255.0 ;
		double	k ;
		if( dn_in <= 63 ) {
			r=0 ; k=255.0/63.0 ; g=(int)(dn_in*k) ; b=255 ;
		} else if( dn_in <= 127 ) {
			r=0 ; g=255 ; k=-255.0/(127.0-63.0) ; b=(int)(((dn_in-63.0)*k)+255.0) ;
		} else if( dn_in <= 191 ) {
			k=255.0/(191.0-127.0) ; r=(int)((dn_in-127.0)*k) ; g=255 ; b=0 ;
		} else {
			r=255; k=-255.0/(255.0-191.0) ; g=(int)(((dn_in-191.0)*k)+255.0) ; b=0 ;
		}
		rgb = ( (uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b ) ;
		color->points[i].rgb = *reinterpret_cast<float *>(&rgb) ;
	}
	pcl::io::savePCDFileASCII( name, *color ) ;
	return ;

}

void DebugID( char name[], struct SegPCD *input ) {

	int num, id_max ;
	num = input->cloud.points.size() ;
	id_max = input->id_max ;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr color( new pcl::PointCloud<pcl::PointXYZRGB> ) ;
	color->height = num ;	color->width = 1 ;	color->is_dense = false ;
	color->points.resize( num ) ;
	//uint8_t r, g, b ;
	unsigned char r, g, b ;
	uint32_t rgb ;
	for( int i=0 ; i<num ; i++ ) {
		color->points[i].x = input->cloud.points[i].x ;
		color->points[i].y = input->cloud.points[i].y ;
		color->points[i].z = input->cloud.points[i].z ;
		double	dn_in = ((double)input->id[i]/(double)id_max) * 255.0 ;
		double	k ;
		if( dn_in <= 63 ) {
			r=0 ; k=255.0/63.0 ; g=(int)(dn_in*k) ; b=255 ;
		} else if( dn_in <= 127 ) {
			r=0 ; g=255 ; k=-255.0/(127.0-63.0) ; b=(int)(((dn_in-63.0)*k)+255.0) ;
		} else if( dn_in <= 191 ) {
			k=255.0/(191.0-127.0) ; r=(int)((dn_in-127.0)*k) ; g=255 ; b=0 ;
		} else {
			r=255; k=-255.0/(255.0-191.0) ; g=(int)(((dn_in-191.0)*k)+255.0) ; b=0 ;
		}
		rgb = ( (uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b ) ;
		color->points[i].rgb = *reinterpret_cast<float *>(&rgb) ;
	}
	pcl::io::savePCDFileASCII( name, *color ) ;
	return ;

}

