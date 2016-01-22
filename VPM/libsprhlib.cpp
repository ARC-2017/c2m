//#include "stdafx.h"

#include "libsprhlib.h"



void InitializeSPRH( const int dist_max, const int angle_max, const int dist_pitch, const int angle_pitch, const int n_sampling, struct SPRH *sprh ){


	fprintf( SE,"dist_max: %d, angle_max: %d, dist_pitch: %d, angle_pitch: %d\n", dist_max, angle_pitch, dist_pitch, angle_pitch );
	int	dist_reso, angle_reso;

	dist_reso = dist_max / dist_pitch;
	angle_reso = angle_max / angle_pitch;

	sprh->angle_max = angle_max;
	sprh->dist_max = dist_max;
	sprh->angle_pitch = angle_pitch;
	sprh->dist_pitch = dist_pitch;
	sprh->angle_reso = angle_reso;
	sprh->dist_reso = dist_reso;
	sprh->n_sampling = n_sampling;
	sprh->gr = cv::Mat::zeros( angle_reso, dist_reso, CV_32F );
	sprh->area = 0;

}

bool LoadSPRH( char name[], struct SPRH *sprh ){

	FILE				*fp;
	char				dum[256];
	errno_t				error_check;

	if( (error_check = fopen_s( &fp, name, "r" )) != 0 ){
		fprintf( stderr,"!!Error %s cannot be load.\n", name );
		return false ;
	};
	if( fscanf( fp, "%s %d", dum, &(sprh->dist_reso) ) < 2 ){
		fprintf( SE,"!!Error in LoadSPRH.\n" );
		return false ;
	}
	if( fscanf( fp, "%s %d", dum, &(sprh->angle_reso) ) < 2 ){
		fprintf( SE,"!!Error in LoadSPRH.\n" );
		return false ;
	}
	if( fscanf( fp, "%s %d", dum, &(sprh->dist_max) ) < 2 ){
		fprintf( SE,"!!Error in LoadSPRH.\n" );
		return false ;
	}
	if( fscanf( fp, "%s %d", dum, &(sprh->angle_max) ) < 2 ){
		fprintf( SE,"!!Error in LoadSPRH.\n" );
		return false ;
	}
	if( fscanf( fp, "%s %d", dum, &(sprh->dist_pitch) ) < 2 ){
		fprintf( SE,"!!Error in LoadSPRH.\n" );
		return false ;
	}
	if( fscanf( fp, "%s %d", dum, &(sprh->angle_pitch) ) < 2 ){
		fprintf( SE,"!!Error in LoadSPRH.\n" );
		return false ;
	}
	if( fscanf( fp, "%s %d", dum, &(sprh->area) ) < 2 ){
		fprintf( SE,"!!Error in LoadSPRH.\n" );
		return false ;
	}
	if( fscanf( fp, "%s %d", dum, &(sprh->n_sampling) ) < 2 ){
		fprintf( SE,"!!Error in LoadSPRH.\n" );
		return false ;
	}

	sprh->gr = cv::Mat::zeros( sprh->angle_reso, sprh->dist_reso, CV_32F );
	double	value;
	for( int jj=0 ; jj<sprh->angle_reso ; jj++ ){
		for( int ii=0 ; ii<sprh->dist_reso ; ii++ ){
			if( fscanf( fp, "%lf", &value ) < 1 ){
				fprintf( SE,"!!Error in LoadSPRH.\n" );
				return false ;
			}
			sprh->gr.at<float>( jj, ii ) = (float)value;
		}
	}

	fclose( fp );

	return true;

}

void CreateSPRH( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud, const int area, const double sampling_rate, struct SPRH *sprh ){

	double		deg2rad, rad2deg;
	deg2rad	=	M_PI/180.0;
	rad2deg =	180.0/M_PI;

	sprh->area = area; 

	sprh->n_sampling = (int)( (double)area * (double)area * sampling_rate);
	fprintf( stderr,"n_sampling: %d\n", sprh->n_sampling );
	

	int					K;
	K = 2;

	// 20150430 秋月追記↓↓↓
	if( K < sprh->area ){ // ペアリングする点が少なすぎる場合は特徴量を作らない．
	// 20150430 秋月追記↑↑↑
		double				max_dist;
		std::vector<int>	pIdx(K);
		std::vector<float>	pD(K);
		std::vector<double>	dist_weight;
		dist_weight.resize( cloud->points.size() );
		pcl::KdTreeFLANN<pcl::PointNormal, flann::L2_Simple<float> > kdtree;
		kdtree.setInputCloud( cloud );

		max_dist = 0.0;
		for( int i=0 ; i<cloud->points.size() ; i++ ){
			if( kdtree.nearestKSearch( cloud->points[i], K, pIdx, pD ) != 0 ){
				dist_weight[i] = (double)sqrt( pD[1] );
				//fprintf( stderr,"pD[0] = %f\n", dist_weight[i] );
				if( max_dist < dist_weight[i]){
					max_dist = dist_weight[i];
				}
			}else{
				dist_weight[i] = 0.0;
			}
		}
		for( int i=0 ; i<dist_weight.size() ; i++ ){
			dist_weight[i] /= max_dist;
		}
		//fprintf( stderr,"max_dist = %lf\n", max_dist );

		//pcl::PointCloud<pcl::PointXYZHSV>::Ptr dist_map (new pcl::PointCloud<pcl::PointXYZHSV> );
		//dist_map->width = cloud->width;
		//dist_map->height = cloud->height;
		//dist_map->points.resize( dist_map->width * dist_map->height );
		//for( int i=0 ; i<dist_map->points.size() ; i++ ){
		//	dist_map->points[i].x = cloud->points[i].x;
		//	dist_map->points[i].y = cloud->points[i].y;
		//	dist_map->points[i].z = cloud->points[i].z;
		//	dist_map->points[i].h = 0.6*( 1.0 - (dist_weight[i] / max_dist) );
		//	dist_map->points[i].s = 1.0;
		//	dist_map->points[i].v = 1.0;
		//}
		//if( CheckSavePCD( dist_map ) == true ){
		//	pcl::io::savePCDFileASCII( "dist_map.pcd", *dist_map );
		//}

		int		idx1, idx2;
		for( int i=0 ; i<sprh->n_sampling ; i++ ){
			idx1 = (int)(((double)rand()/(double)RAND_MAX) * (double)(cloud->points.size()-1));
			idx2 = (int)(((double)rand()/(double)RAND_MAX) * (double)(cloud->points.size()-1));

			//点ペアの距離計算
			double3 p, q;
			double	dist;
			p.x = cloud->points[idx1].x;
			p.y = cloud->points[idx1].y;
			p.z = cloud->points[idx1].z;

			q.x = cloud->points[idx2].x;
			q.y = cloud->points[idx2].y;
			q.z = cloud->points[idx2].z;
			AkiCalcDistance( p, q, &dist );

			if( dist < sprh->dist_max ){
				//点ペア法線の角度計算
				double3 np, nq;
				double	angle;
				np.x = cloud->points[idx1].normal_x;
				np.y = cloud->points[idx1].normal_y;
				np.z = cloud->points[idx1].normal_z;

				nq.x = cloud->points[idx2].normal_x;
				nq.y = cloud->points[idx2].normal_y;
				nq.z = cloud->points[idx2].normal_z;
				AkiInnerProduct( np, nq, &angle );
				angle = rad2deg * (double)acos( (float)angle );

				//fprintf( stderr,"p(%.2lf, %.2lf, %.2lf)\n", p.x, p.y, p.z );
				//fprintf( stderr,"q(%.2lf, %.2lf, %.2lf)\n", q.x, q.y, q.z );
				//fprintf( stderr,"dist: %.2lf\n", dist );

				//fprintf( stderr,"np(%.2lf, %.2lf, %.2lf)\n", np.x, np.y, np.z );
				//fprintf( stderr,"nq(%.2lf, %.2lf, %.2lf)\n", nq.x, nq.y, nq.z );
				//fprintf( stderr,"angle: %.2lf\n", angle );

				//投票位置の決定
				int vote_dist, vote_angle;
				vote_dist = (int)( dist / (double)sprh->dist_pitch);
				if( vote_dist == sprh->dist_reso ) vote_dist--;  
				vote_angle = (int)( angle / (double)sprh->angle_pitch);
				if( vote_angle == sprh->angle_reso ) vote_angle--;  

				//fprintf( stderr,"vote( %d, %d )\n", vote_dist, vote_angle );
				//fprintf( stderr,"dist_w1: %.2lf, dist_w2: %.2lf\n", dist_weight[idx1], dist_weight[idx2] );

				// 点ペアの投票
				//重みなし
				//sprh->gr.at<float>( vote_angle, vote_dist ) += 1.0;
				//重みあり
				sprh->gr.at<float>( vote_angle, vote_dist ) += (float)(dist_weight[idx1] * dist_weight[idx2]);

				//fprintf( stderr,"vote ok\n" );
			}

		}

	}
	// 合計値の算出
	double	sum;
	sum = 0.0;
	for( int j=0 ; j<sprh->gr.rows ; j++ ){
		for( int i=0 ; i<sprh->gr.cols ; i++ ){
			sprh->gr.at<float>( j, i ) += 0.1;
			sum += sprh->gr.at<float>( j, i );
		}
	}
	// 合計1に正規化
	for( int j=0 ; j<sprh->gr.rows ; j++ ){
		for( int i=0 ; i<sprh->gr.cols ; i++ ){
			sprh->gr.at<float>( j, i ) /= (float)sum;
		}
	}
}

// 特徴量の可視化
void SaveSPRHAsImg( char name[], struct SPRH *sprh ){

	fprintf( stderr,"%s\n", name );
	fprintf( stderr,"size( %d, %d )\n", sprh->gr.cols, sprh->gr.rows );
	double	max;
	cv::Mat	sprh_img;
	sprh_img = cv::Mat::zeros( sprh->gr.rows, sprh->gr.cols, CV_8U );
	max = 0.0;
	for( int j=0 ; j<sprh->gr.rows ; j++ ){
		for( int i=0 ; i<sprh->gr.cols ; i++ ){
			if( max < sprh->gr.at<float>( j, i ) ){
				max = sprh->gr.at<float>( j, i );
			}
		}
	}
	for( int j=0 ; j<sprh->gr.rows ; j++ ){
		for( int i=0 ; i<sprh->gr.cols ; i++ ){
			sprh_img.at<uchar>( j, i ) = (unsigned char)(255.0 * sprh->gr.at<float>( j, i ) / max );
		}
	}
	cv::Mat	sprh_img_big;
	sprh_img_big = cv::Mat::zeros( (int)(4.0*sprh_img.rows), (int)(4.0*sprh_img.cols), sprh_img.type() );
	cv::resize( sprh_img, sprh_img_big, sprh_img_big.size(), 4.0, 4.0, cv::INTER_NEAREST );

	cv::imwrite( name, sprh_img_big );
}

// 特徴量の保存
void SaveSPRH( char name[], struct SPRH *sprh ){ 

	FILE	*fp;
	fp = fopen( name, "w" );
	fprintf( fp,"sprh_width: %d\n", sprh->gr.cols );
	fprintf( fp,"sprh_height: %d\n", sprh->gr.rows );
	fprintf( fp,"dist_max: %d\n", sprh->dist_max );
	fprintf( fp,"angle_max: %d\n", sprh->angle_max );
	fprintf( fp,"dist_pitch: %d\n", sprh->dist_pitch );
	fprintf( fp,"angle_pitch: %d\n", sprh->angle_pitch );
	fprintf( fp,"area: %d\n", sprh->area );
	fprintf( fp,"n_sampling: %d\n", sprh->n_sampling );

	for( int j=0 ; j<sprh->gr.rows ; j++ ){
		for( int i=0 ; i<sprh->gr.cols ; i++ ){
			fprintf( fp, "%lf\n", sprh->gr.at<float>( j, i ) );
		}
	}
	fclose( fp );
}

// 視点の設定．view_pitchは緯度方向の角度ピッチ
void SetViewpoint( int view_pitch, std::vector<double3> *viewpoint ){

	double3					tmp_vp;
	int						n_dir;
	double					deg2rad, rad2deg;

	deg2rad = M_PI/180.0;
	rad2deg = 180.0/M_PI;

	n_dir = 360 / view_pitch;
	tmp_vp.x = 0.0; tmp_vp.y = 1.0; tmp_vp.z = 0.0;  //上
	viewpoint->push_back( tmp_vp );
	tmp_vp.x = 0.0; tmp_vp.y = -1.0; tmp_vp.z = 0.0; //下
	viewpoint->push_back( tmp_vp );

	double	rot[3][3];
	double3 rot_tmp_vp;
	tmp_vp.x = 0.0; tmp_vp.y = sin( 45.0*deg2rad ); tmp_vp.z = cos( 45.0*deg2rad );
	for( int i=0 ; i<n_dir ; i++ ){
		AkiRollPitchYaw2Rot( 0.0, (double)(deg2rad*i*view_pitch), 0.0, rot );
		Aki3DMatrixx3DPoint( rot, tmp_vp, &rot_tmp_vp );
		viewpoint->push_back( rot_tmp_vp );
	}
	tmp_vp.x = 0.0; tmp_vp.y = 0.0; tmp_vp.z = 1.0;
	for( int i=0 ; i<n_dir ; i++ ){
		AkiRollPitchYaw2Rot( 0.0, (double)(deg2rad*i*view_pitch), 0.0, rot );
		Aki3DMatrixx3DPoint( rot, tmp_vp, &rot_tmp_vp );
		viewpoint->push_back( rot_tmp_vp );
	}
	tmp_vp.x = 0.0; tmp_vp.y = sin( -45.0*deg2rad ); tmp_vp.z = cos( -45.0*deg2rad );
	for( int i=0 ; i<n_dir ; i++ ){
		AkiRollPitchYaw2Rot( 0.0, (double)(deg2rad*i*view_pitch), 0.0, rot );
		Aki3DMatrixx3DPoint( rot, tmp_vp, &rot_tmp_vp );
		viewpoint->push_back( rot_tmp_vp );
	}
}

//void MatchingSPRH( const struct SPRH *sprh1, const struct SPRH *sprh2, double *score ){ 
//
//
//	double tmp_score;
//	tmp_score = 0.0;
//	for( int j=0 ; j<sprh1->gr.rows ; j++ ){
//		for( int i=0 ; i<sprh1->gr.cols ; i++ ){
//			if( sprh1->gr.at<float>( j, i ) < sprh2->gr.at<float>( j, i ) ){
//				tmp_score += sprh1->gr.at<float>( j, i );
//			}else{
//				tmp_score += sprh2->gr.at<float>( j, i );
//			}
//		}
//	}
//
//	//double	area_score;
//	//if( sprh1->area < sprh2->area ){
//	//	area_score = (double)((double)sprh1->area / (double)sprh2->area);
//	//}else{
//	//	area_score = (double)((double)sprh2->area / (double)sprh1->area);
//	//}
//
//	//fprintf( stderr,"gr_score: %.2lf, area_score: %.2lf\n", tmp_score, area_score );
//	//*score = (tmp_score + area_score) / 2.0 ;
//	*score = tmp_score;
//}

void MatchingSPRH2( const struct SPRH *sprh_m, const struct SPRH *sprh_s, double *score ){ 


	double tmp_score;
	tmp_score = 0.0;
	// KL divergence
	for( int j=0 ; j<sprh_m->gr.rows ; j++ ){
		for( int i=0 ; i<sprh_m->gr.cols ; i++ ){
			if( (sprh_m->gr.at<float>( j, i )!=0.0) && (sprh_s->gr.at<float>( j, i )!=0.0) ){
				tmp_score += (sprh_m->gr.at<float>( j, i ) - sprh_s->gr.at<float>( j, i ) )  
					         * log( sprh_m->gr.at<float>( j, i ) / sprh_s->gr.at<float>( j, i ) );
			}
		}
	}
	// SSD
	//for( int j=0 ; j<sprh_m->gr.rows ; j++ ){
	//	for( int i=0 ; i<sprh_m->gr.cols ; i++ ){
	//		tmp_score += (sprh_m->gr.at<float>( j, i ) - sprh_s->gr.at<float>( j, i )) *
	//					 (sprh_m->gr.at<float>( j, i ) - sprh_s->gr.at<float>( j, i ));
	//	}
	//}

	*score = tmp_score;
}

bool RecgSimple( const int num_of_seg, int *itemIdx, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double *SimpleworkPos, double *SimpleScore,
				 int *CenterIdx, std::vector<int>& cloud_cluster_idx ){

	// 時間計測の開始
	AkiStartTimer();
	AkiTimeSetFlag();

	// pointXYZRGB -> pointXYZ
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
	cloudXYZ->points.resize( cloud->points.size() );
	cloudXYZ->width = cloud->width;
	cloudXYZ->height = cloud->height;
	for( int i=0 ; i<cloudXYZ->points.size() ; i++ ){
		cloudXYZ->points[i].x = cloud->points[i].x;
		cloudXYZ->points[i].y = cloud->points[i].y;
		cloudXYZ->points[i].z = cloud->points[i].z;
	}


	// 距離画像生成用パラメータ
	int				img_cols, img_rows;
	struct double3	transZero;
	double			pixel_pitch;
	FILE			*fp, *fp_ch, *fp_log;
	fopen_s( &fp_log, "log_Simple.txt", "w" );

	img_cols = IMG_COLS;
	img_rows = IMG_ROWS;
	pixel_pitch = PIXEL_PITCH;  //1画素当たりのピッチ
	transZero.x = TRANS_ZERO_X;//オフセット
	transZero.y = TRANS_ZERO_Y; 
	transZero.z = TRANS_ZERO_Z;

	int		width, height;
	int		N_SPRH;

	width = 1280;
	height = 960;
	N_SPRH = 26;
	fprintf( fp_log,"RecgSimple\n" );
	fprintf( fp_log,"itemIdx: %d\n", *itemIdx );

	//New
	//CHファイルの読み込み
	fopen_s( &fp_ch, "log_Simple_CH.txt", "w" ) ;
	fprintf( fp_ch, "Loading ch files...\n" ) ;

	char	chfname[FNL], name_ch[FNL];
	std::vector<CH>		ch_m;

	ch_m.resize( N_SPRH );
	for( int i=0 ; i<N_SPRH ; i++ ){
		sprintf( chfname,"%s%d\\%d%s", MODEL_PATH, *itemIdx, i, CH_NAME );
		if( LoadCH( chfname, &ch_m[i] ) == false ){
			fprintf( fp_ch,"!!ERROR %s cannot be read.\n", chfname );
			fclose( fp_log );
			return false;
		}
		//Debug
		for( int j=0, l=(ch_m[i].level)*3 ; j<l ; j++ ) {
			fprintf( fp_ch, "ch_m.df[%d]: %lf\n", j, ch_m[i].dh[j] ) ;
		}
	}
	fclose( fp_ch ) ;
	AkiTimeLogFromFlag( "Load CH files" );
	AkiTimeSetFlag();


	//SPRHファイルの読み込み
	fprintf( fp_log,"Loading sprh files...\n" );
	char					sprhfname[FNL], name[FNL];
	std::vector<SPRH>		sprh_m;

	sprintf( sprhfname,"%s%d\\%d%s", MODEL_PATH, *itemIdx, *itemIdx, SPRH_NAME );

	sprh_m.resize( N_SPRH );
	for( int i=0 ; i<N_SPRH ; i++ ){

		sprintf( name,"%s%d", sprhfname, i );
		fprintf( fp_log,"Loading %s ...\n", name );
		if( LoadSPRH( name, &sprh_m[i] ) == false ){
			fprintf( fp_log,"!!ERROR %s cannot be read.\n", name );
			fclose( fp_log );
			return false;
		};

		// エラー処理．SPRHの次元がすべて同じかチェックする．
		if( i != 0 ){
			if( sprh_m[0].gr.cols != sprh_m[i].gr.cols ){
				fprintf( fp_log,"!!ERROR Size of dist_reso is different #%d.\n", i ); 
				fclose( fp_log );
				return false;
			}
			if( sprh_m[0].gr.rows != sprh_m[i].gr.rows ){
				fprintf( fp_log,"!!ERROR Size of angle_reso is different #%d.\n", i ); 
				fclose( fp_log );
				return false;
			}
		}

	}

	//// デバッグ
	//for( int i=0 ; i<N_SPRH ; i++ ){
	//	sprintf( name,"sprh%d.bmp", i );
	//	// 特徴量の保存（画像）
	//	SaveSPRHAsImg( name, &sprh_m[i] );
	//}
	AkiTimeLogFromFlag( "Load SPRH files" );
	AkiTimeSetFlag();

	//
	// 前処理の開始
	//

	//Add
	//ダウンサンプリング
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	fprintf( fp_log,"Down sampling of scene data.\n" );
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize(S_LEAF, S_LEAF, S_LEAF);
	sor.filter(*cloud_filtered);
	// シーンデータのアウトライアの削除
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clean (new pcl::PointCloud<pcl::PointXYZRGB>);
	fprintf( fp_log,"Outlier removal of scene data.\n");
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> ssor;
	ssor.setInputCloud( cloud_filtered );
	ssor.setMeanK( 50 );  //近くの50点を使って推定
	ssor.setStddevMulThresh( 1.0 );
	ssor.filter( *cloud_clean );

	for( int i=0 ; i<N_OUTLIER_REMOVAL-1 ; i++ ){
		ssor.setInputCloud( cloud_clean );
		ssor.filter( *cloud_clean );
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cleanXYZ (new pcl::PointCloud<pcl::PointXYZ>) ;
	pcl::copyPointCloud( *cloud_clean, *cloud_cleanXYZ ) ;

	AkiTimeLogFromFlag( "Removing outlier of an input scene" );
	AkiTimeSetFlag();
	fprintf( fp_log,"%d times\n", N_OUTLIER_REMOVAL );
	fprintf( fp_log," Npoints[%d] -> Npoints[%d]\n", cloud_filtered->points.size(), cloud_clean->points.size() );

	//Add RGB用
	// 単一視点の点群データの生成(主に二重になっている計測データの削除)
	pcl::PointCloud<int> visibleIdx;
	double3 viewpoint;
	viewpoint.x = 0.0;
	viewpoint.y = 0.0;
	viewpoint.z = 500.0;
	HPR_operator( cloud_cleanXYZ, &viewpoint, 2.5, visibleIdx );
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visible (new pcl::PointCloud<pcl::PointXYZRGB>);
	CopyPCD( cloud_clean, visibleIdx, cloud_visible ) ;
#if OUTPUT2
	if( CheckSavePCD( cloud_visible ) == true ){
		sprintf( name,"cloud_visible.pcd" );
		pcl::io::savePCDFile( name, *cloud_visible ); 
	}
#endif

	AkiTimeLogFromFlag( "Removing hidden points" );
	AkiTimeSetFlag();
	fprintf( fp_log," Npoints[%d] -> Npoints[%d]\n", cloud_clean->points.size(), cloud_visible->points.size() );

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_visibleXYZ (new pcl::PointCloud<pcl::PointXYZ>) ;
	pcl::copyPointCloud( *cloud_visible, *cloud_visibleXYZ ) ;

	//シーンのポイントクラウドの法線推定（パラメータ：N_NEIGHBORは推定に使うデータ点の範囲）
	pcl::PointCloud<pcl::PointNormal>::Ptr tmp_pnt_nrm (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr pnt_nrm (new pcl::PointCloud<pcl::PointNormal>);
	fprintf( fp_log,"Estimating surface normal vectors of an input scene.\n");
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	fprintf( fp_log," View point( %.2lf, %.2lf, %.2lf )\n", VIEWPOINT_X, VIEWPOINT_Y, VIEWPOINT_Z );
	ne.setViewPoint( VIEWPOINT_X, VIEWPOINT_Y, VIEWPOINT_Z );
	ne.setInputCloud (cloud_visibleXYZ);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (N_NEIGHBOR);
	ne.compute (*cloud_normals);
	pcl::concatenateFields (*cloud_visibleXYZ, *cloud_normals, *tmp_pnt_nrm );

	// NaN remove
	std::vector<int> sceneIdx;
	removeNaNNormalsFromPointCloud( *tmp_pnt_nrm, *pnt_nrm, sceneIdx );
	
	AkiTimeLogFromFlag( "Estimating normal vectors of an input scene" );
	AkiTimeSetFlag();
	fprintf( fp_log," Nan remove\n" );
	fprintf( fp_log," Npoints[%d] -> Npoints[%d]\n", tmp_pnt_nrm->points.size(), pnt_nrm->points.size() );

	//New
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pnt_rgb( new pcl::PointCloud<pcl::PointXYZRGB> ) ;
	pnt_rgb->height = sceneIdx.size() ;	pnt_rgb->width = 1 ;
	pnt_rgb->resize( sceneIdx.size() ) ;
	for( int i=0, l=sceneIdx.size() ; i<l ; i++ ) {
		pnt_rgb->points[i].x = cloud_visible->points[sceneIdx[i]].x ;
		pnt_rgb->points[i].y = cloud_visible->points[sceneIdx[i]].y ;
		pnt_rgb->points[i].z = cloud_visible->points[sceneIdx[i]].z ;
		pnt_rgb->points[i].r = cloud_visible->points[sceneIdx[i]].r ;
		pnt_rgb->points[i].g = cloud_visible->points[sceneIdx[i]].g ;
		pnt_rgb->points[i].b = cloud_visible->points[sceneIdx[i]].b ;
	}

	// セグメンテーション
	std::vector<std::vector<int>> segIdx;
	fprintf( fp_log,"segmentation...\n" );
	// ビン内の物体数が0（セグメント数が0）のときのエラー処理を追加 2015.05.09 武井
	if( !num_of_seg ) {
		fprintf( fp_log,"!!ERROR Number of object in the bin: 0\n" ); 
		fclose( fp_log ) ;
		return false;
	}
	//extracteCluster( pnt_nrm, 2, 100, 1.0, segIdx ) ;
	fprintf( fp_log,"...OK.\n" );
	//std::vector<std::vector<int>> segIdx2;
	SegmentDomain( pnt_rgb, num_of_seg, N_NEI_C, TH_C, segIdx ) ;
	fprintf( fp_log,"...OK.\n" );

	AkiTimeLogFromFlag( "Segmentation" );
	AkiTimeSetFlag();

	//New
	// カラーヒストグラムの生成を書く
	int	level, max_intensity ;
	std::vector<struct CH>	colorHistogram ;
	colorHistogram.resize( segIdx.size() ) ;
	level = ch_m[0].level ;	max_intensity = ch_m[0].max ;
	fprintf( stderr,"    level: %d\n\n\n", level );
	for( int i=0 ; i<segIdx.size() ; i++ ){
		//セグメントデータのコピー
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segRGB( new pcl::PointCloud<pcl::PointXYZRGB> ) ;
		CopyPCD( pnt_rgb, segIdx[i], cloud_segRGB );
		//セグメントデータの保存
#if OUTPUT2
		if( CheckSavePCD( cloud_segRGB ) == true ){
			sprintf( name,"cloud_segRGB_simple%d-%d.pcd", num_of_seg, i+1 );
			pcl::io::savePCDFile( name, *cloud_segRGB ); 	
		}
#endif
		//特徴量の計算
		fprintf( SE, "InitializeCH\n" ) ;
		InitializeCH( level, max_intensity, &(colorHistogram[i]) ) ;
		fprintf( SE, "CreateCH\n" ) ;
		CreateCHPCL( cloud_segRGB, &(colorHistogram[i]) ) ;
#if OUTPUT2
		//Debug
		sprintf( name, "colorHistogram_seg%d", i );
		SaveCH( name, &(colorHistogram[i]) ) ;
#endif
	}
	AkiTimeLogFromFlag( "Create scene CH" );
	AkiTimeSetFlag();

	// シーンからのSPRHの算出
	std::vector<struct SPRH>		sprh_s;
	sprh_s.resize( segIdx.size() );
	for( int i=0 ; i<segIdx.size() ; i++ ){
		// セグメントデータのコピー
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_seg (new pcl::PointCloud<pcl::PointNormal>);
		CopyPCD( pnt_nrm, segIdx[i], cloud_seg );
		// セグメントデータの保存
#if OUTPUT2
		// シンプルアルゴリズムのセグメンテーション結果を保存（分割数ごとに別ファイルに保存） 2015.05.09 武井
		if( CheckSavePCD( cloud_seg ) == true ){
			sprintf( name,"cloud_seg_simple%d-%d.pcd", num_of_seg, i+1 );
			pcl::io::savePCDFile( name, *cloud_seg ); 	
		}
		cv::Mat seg_img, seg_img2;
		seg_img = cv::Mat::zeros( img_rows, img_cols, CV_8U );
		PCDNormal2CVMat( cloud_seg, pixel_pitch, transZero, 1, &seg_img );
		cv::flip( seg_img, seg_img2, 0 );
		sprintf( name,"img_seg_simple%d-%d.bmp", num_of_seg, i+1 );
		cv::imwrite( name, seg_img2 );
#endif

		// 特徴量の計算
		InitializeSPRH( sprh_m[0].dist_max, sprh_m[0].angle_max, 
						sprh_m[0].dist_pitch, sprh_m[0].angle_pitch,
						sprh_m[0].n_sampling, &sprh_s[i] );
		//CreateSPRH( cloud_bd, (int)cloud_seg->points.size(), &sprh_s[i] );
		fprintf( fp_log,"N point of segment: %d\n", cloud_seg->points.size() );
		CreateSPRH( cloud_seg, (int)cloud_seg->points.size(), 0.05, &sprh_s[i] );

#if OUTPUT2
		//特徴量の可視化
		sprintf( name,"scene_sprh%d.bmp", i );
		SaveSPRHAsImg( name, &sprh_s[i] );
#endif
		
	}
	AkiTimeLogFromFlag( "Create scene SPRH" );
	AkiTimeSetFlag();

	// New
	// SPRH間の類似度計算
	// + ColorHistogram間の類似度計算
	double				score_ch, score_sprh, score_total ;
	double				score, max_score ;
	int					max_model_id, max_seg_id;
	
	max_score = 99999999.0 ;
	max_model_id = max_seg_id = 0;
	for( int j=0 ; j<segIdx.size() ; j++ ){		//segment数のループ

		// 20150430秋月追記↓↓↓
		fprintf( fp_log,"pnt_rgb->points.size()/segIdx.size() = %d\n", pnt_rgb->points.size()/segIdx.size() );
		fprintf( fp_log,"sprh_s[%d].area = %d\n", j, sprh_s[j].area );
		if( (int)(VALID_SEGMENT_SIZE*(double)pnt_rgb->points.size()) < sprh_s[j].area ){ // アピアランスの面積が小さすぎる場合は無視
		// 20150430秋月追記↑↑↑

			for( int i=0 ; i<N_SPRH ; i++ ){		//modelのSPRHのループ
				MatchingCH( &ch_m[i], &(colorHistogram[j]), &score_ch ) ;
				MatchingSPRH2( &sprh_m[i], &sprh_s[j], &score_sprh );  // KL divergence
				fprintf( fp_log,"seg id: %d, sprh_id: %d, color_score: %lf, sprh_score: %lf\n", j, i, score_ch, score_sprh );

				// Color + Shape の類似度算出
				score_total = score_ch * score_sprh ;

				if( max_score > score_total ){
					max_score = score_total;
					max_model_id = i;
					max_seg_id = j ;
				}

			}
		}else{
			fprintf( fp_log,"# of point of segment is too low. -> skip\n" );
		}
	}

	AkiTimeLogFromFlag( "Matching" );
	AkiTimeSetFlag();


	fprintf( fp_log,"Result: \n" );
	fprintf( fp_log," seg id: %d, sprh_id: %d, score: %lf\n", max_seg_id, max_model_id, max_score );
	

	// 識別結果のセグメントの重心計算
	pcl::PointXYZ	center;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_seg_result (new pcl::PointCloud<pcl::PointNormal>);
	CopyPCD( pnt_nrm, segIdx[max_seg_id], cloud_seg_result );
	CalcCenter( cloud_seg_result, &center );

	// 認識結果
	double	normalized_score;
	SimpleworkPos[0] = center.x;
	SimpleworkPos[1] = center.y;
	SimpleworkPos[2] = center.z;
	if( 1.0 < max_score ){
		max_score = 1.0;
	}
	normalized_score = 1.0 - max_score;
	*SimpleScore = normalized_score;

	// クラスタの抽出
	pcl::PointXYZ CenterOfCluster;
	pcl::PointXYZRGB seed;
	CalcCenter( cloud_seg_result, &CenterOfCluster );
	seed.x = CenterOfCluster.x;
	seed.y = CenterOfCluster.y;
	seed.z = CenterOfCluster.z;
	
	pcl::KdTreeFLANN<pcl::PointXYZRGB, flann::L2_Simple<float> > c_kdtree;
	c_kdtree.setInputCloud( cloud );
	std::vector<int> c_pIdx;
	std::vector<float> c_pSd;
	c_pIdx.resize(1);
	c_pSd.resize(1);
	c_pIdx[0] = 0;
	if ( c_kdtree.nearestKSearch( seed, 1, c_pIdx, c_pSd) > 0 ){
		*CenterIdx = c_pIdx[0];
	}
	// シーンにおける物体クラスタの抽出
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_source (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	
	CloudSubtractionInv( cloudXYZ, cloud_seg_result, S_LEAF*1.5, cloud_seg_source, cloud_cluster_idx );

#if OUTPUT2

	time_t time_s2e;
	time_s2e = AkiTimeFromStart();
	fopen_s( &fp, "log_Simple_time.txt", "a" );
	fprintf( fp,"%d\n", time_s2e );
	fclose( fp );
	fopen_s( &fp, "log_Simple_score.txt", "a" );
	fprintf( fp,"%lf\n", max_score );
	fclose( fp );

	// 最も似ていた特徴量の保存（画像）
	sprintf( name,"nearest_sprh_id%d_score%.2lf.bmp", max_model_id, max_score  );
	SaveSPRHAsImg( name, &sprh_m[max_model_id] );

	//中間データの出力 .bmp
	fprintf( fp_log,"Save an input image as img_scene.bmp.\n");
	cv::Mat scene_img, scene_img2;
	scene_img = cv::Mat::zeros( img_rows, img_cols, CV_8U );
	PCDXYZ2CVMat( cloudXYZ, pixel_pitch, transZero, 0, &scene_img );
	cv::flip( scene_img, scene_img2, 0 );
	cv::imwrite( "img_scene_simple.bmp", scene_img2 );

	//重心位置にマークした画像の保存
	fprintf( fp_log,"Save a marked iamge as scoremap_centroid.bmp.\n");
	double3 center_d3;
	center_d3.x = center.x; center_d3.y = center.y; center_d3.z = center.z;
	cv::Mat centroid_img, centroid_img2;
	centroid_img = cv::Mat::zeros( img_rows, img_cols, CV_8UC3 );
	centroid_img2 = cv::Mat::zeros( img_rows, img_cols, CV_8UC3 );
	PCD2CVMatDrawCentroid( center_d3, pixel_pitch, transZero, scene_img, &centroid_img );
	char	score_c[FNL];
	cv::flip( centroid_img, centroid_img2, 0 );
	sprintf( score_c,"score: %.3lf", normalized_score );
	cv::putText( centroid_img2, score_c, cv::Point( 30, 60 ), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,0), 2, CV_AA);
	sprintf( score_c,"%s%d", sprhfname, max_model_id );
	cv::putText( centroid_img2, score_c, cv::Point( 30, 30 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1, CV_AA);
	cv::imwrite( "scoremap_centroid_simple.bmp", centroid_img2 );

	//中間データの出力 .pcd
	if( CheckSavePCD( cloud_filtered ) == true ){
		fprintf( fp_log,"Save cloud_filtered_scene.pcd.\n");
		pcl::io::savePCDFileASCII( "cloud_filtered_scene_simple.pcd", *cloud_filtered );
	}
	if( CheckSavePCD( cloud_clean ) == true ){
		fprintf( fp_log,"Save cloud_clean_scene.pcd.\n");
		pcl::io::savePCDFileASCII( "cloud_clean_scene_simple.pcd", *cloud_clean );
	}
	if( CheckSavePCD( pnt_nrm ) == true ){
		fprintf( fp_log,"Save cloud_preprocessed_scene_simple.pcd.\n");
		pcl::io::savePCDFileASCII( "cloud_preprocessed_scene_simple.pcd", *pnt_nrm );
	}
	if( CheckSavePCD( cloud ) == true ){
		pcl::io::savePCDFileASCII( "cloud_sub_simple.pcd", *cloud );
	}



#endif

	//New
	for( int i=0 ; i<N_SPRH ; i++ ){
		DeleteCH( &(ch_m[i]) ) ;
	}	
	for( int i=0 ; i<segIdx.size() ; i++ ){
		DeleteCH( &(colorHistogram[i]) ) ;
	}

	fclose( fp_log );

	return true;

}

void extracteCluster( pcl::PointCloud<pcl::PointNormal>::Ptr input, int num_of_object, int max_it, float th_var, std::vector<std::vector<int>>& segIdx ){

	//Double-bin用のアルゴリズム．
	//点群の存在範囲の抽出
	segIdx.resize( num_of_object );
	int	l=(int)(input->points.size()) ;
	pcl::PointNormal min, max;
	double3			 p1, p2 ;
	min.x = max.x = input->points[0].x ;
	min.y = max.y = input->points[0].y ;
	min.z = max.z = input->points[0].z ;
	for( int i=1 ; i<l ; i++ ) {
		float x, y, z ;
		x = input->points[i].x ;	y = input->points[i].y ;	z = input->points[i].z ;
		if( x<min.x )	min.x = x ;
		if( y<min.y )	min.y = y ;
		if( z<min.z )	min.z = z ;
		if( max.x<x )	max.x = x ;
		if( max.y<y )	max.y = y ;
		if( max.z<z )	max.z = z ;
	}
	fprintf( SE,"max:( %.2lf, %.2lf, %.2lf )\n", max.x, max.y, max.z );
	fprintf( SE,"min:( %.2lf, %.2lf, %.2lf )\n", min.x, min.y, min.z );
	p1.x = (double)(((max.x-min.x)/3.0)+min.x) ;		p1.y = (double)(((max.y-min.y)/3.0)+min.y) ;		p1.z = (double)(((max.z-min.z)/3.0)+min.z) ;
	p2.x = (double)(((max.x-min.x)/3.0)*2.0+min.x) ;	p2.y = (double)(((max.y-min.y)/3.0)*2.0+min.y) ;	p2.z = (double)(((max.z-min.z)/3.0)*2.0+min.z) ;

	fprintf( SE,"p1:( %.2lf, %.2lf, %.2lf )\n", p1.x, p1.y, p1.z );
	fprintf( SE,"p2:( %.2lf, %.2lf, %.2lf )\n", p2.x, p2.y, p2.z );

	double3 tmp_p1, tmp_p2 ;
	double	var1, var2 ;
	for( int n=0 ; n<max_it ; n++ ) {
		fprintf( SE,"iteration:%d\n", n );
		segIdx[0].clear();
		segIdx[1].clear();
		for( int i=0 ; i<l ; i++ ) {
			double	e1, e2;
			double3	tmp_point;
			tmp_point.x = (double)input->points[i].x ; 
			tmp_point.y = (double)input->points[i].y ; 
			tmp_point.z = (double)input->points[i].z ; 
			AkiCalcDistance( tmp_point, p1, &e1 );	
			AkiCalcDistance( tmp_point, p2, &e2 );	

			if( e1<e2 ) {	// 重心点と各点の距離を測って，入力点群を二つに分ける．
				segIdx[0].push_back( i );
			}else{
				segIdx[1].push_back( i );
			}
		}

		// 振り分けられた点群の重心を算出 -> tmp_p1, tmp_p2
		tmp_p1.x = tmp_p1.y = tmp_p1.z = 0.0 ;
		for( int ii=0 ; ii<segIdx[0].size() ; ii++ ) {
			tmp_p1.x += input->points[segIdx[0][ii]].x ;	tmp_p1.y += input->points[segIdx[0][ii]].y ;	tmp_p1.z += input->points[segIdx[0][ii]].z ;
		}
		tmp_p1.x /= (double)segIdx[0].size() ;	tmp_p1.y /= (double)segIdx[0].size() ;	tmp_p1.z /= (double)segIdx[0].size() ;


		tmp_p2.x = tmp_p2.y = tmp_p2.z = 0.0 ;
		for( int ii=0 ; ii<segIdx[1].size() ; ii++ ) {
			tmp_p2.x += input->points[segIdx[1][ii]].x ;	tmp_p2.y += input->points[segIdx[1][ii]].y ;	tmp_p2.z += input->points[segIdx[1][ii]].z ;
		}
		tmp_p2.x /= (double)segIdx[1].size() ;	tmp_p2.y /= (double)segIdx[1].size() ;	tmp_p2.z /= (double)segIdx[1].size() ;

		AkiCalcDistance( p1, tmp_p1, &var1 );
		AkiCalcDistance( p2, tmp_p2, &var2 );

		fprintf( SE,"seg1: %d, seg2: %d\n", segIdx[0].size(), segIdx[1].size() );
		fprintf( SE,"dist1: %.2lf, dist2: %.2lf\n", var1, var2 );

		// 重心点の移動距離が閾値以下だったら終わり
		if( var1<=th_var && var2<=th_var ) break ;

		p1.x = tmp_p1.x ;	p1.y = tmp_p1.y ;	p1.z = tmp_p1.z ;
		p2.x = tmp_p2.x ;	p2.y = tmp_p2.y ;	p2.z = tmp_p2.z ;
	}


}
