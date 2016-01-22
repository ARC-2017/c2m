/////////////////////////////////////////////////////////////////////////////
//
//	librecglib.cpp:
//
//  Shuichi AKIZUKI
//
//	(C) 2015 ISL, Chukyo University All rights reserved.
//
//  Note:
//		大学側認識処理プログラムの共通的な関数群の内容
//	
//		2015.04.20
//		コメント文をいくつか追加．
//		2015.04.30
//		村田君のコードを反映．
//
//////////////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "recg.h"
#include "librecglib.h"
#include "bin_size.h"
#include "algorithm_table.h"

using namespace cv;

#define seg_noise_reduction (0) //セグメンテーションを使ったノイズ除去のON/OFF


void ShowRecognitionResult( int method, double *workPos, double *score ){

	if( method == METHOD_WHF ){
		fprintf( SE,"\n" );
		fprintf( SE,"/--------------------------------------------/\n" );
		fprintf( SE,"Result of Weighted Hough Forest\n" );

		fprintf( SE,"Work Position:\n" );
		fprintf( SE,"( " );
		for( int i=0 ; i<12 ; i++ ){
			fprintf( SE,"%.1lf ", workPos[i] );
		}
		fprintf( SE,")\n" );

	}else if( method == METHOD_VPM ){
		fprintf( SE,"\n" );
		fprintf( SE,"/--------------------------------------------/\n" );
		fprintf( SE,"Result of Vector Pair Matching\n" );
		fprintf( SE,"Work Position:\n" );
		fprintf( SE,"    | %.3lf %.3lf %.3lf |\n", workPos[0], workPos[1], workPos[2] );
		fprintf( SE,"R = | %.3lf %.3lf %.3lf |\n", workPos[3], workPos[4], workPos[5] );
		fprintf( SE,"    | %.3lf %.3lf %.3lf |\n", workPos[6], workPos[7], workPos[8] );
		fprintf( SE,"t = < %.3lf %.3lf %.3lf >\n", workPos[9], workPos[10], workPos[11] );

		fprintf( SE,"( " );
		for( int i=0 ; i<12 ; i++ ){
			fprintf( SE,"%.1lf ", workPos[i] );
		}
		fprintf( SE,")\n" );

	}else if( method == METHOD_SIMPLE ){
		fprintf( SE,"\n" );
		fprintf( SE,"/--------------------------------------------/\n" );
		fprintf( SE,"Result of Simple method (SPRH)\n" );
		fprintf( SE,"Work Position:\n" );
		fprintf( SE,"( " );
		for( int i=0 ; i<12 ; i++ ){
			fprintf( SE,"%.1lf ", workPos[i] );
		}
		fprintf( SE,")\n" );
	}
	fprintf( SE,"Score: %lf\n", *score );
	fprintf( SE,"/--------------------------------------------/\n" );
	fprintf( SE,"\n" );


}

void CopyWorkPos( double *workPos1, int num, double *workPos2 ){

	for( int i=0 ; i<num ; i++ ){
		workPos2[i] = workPos1[i];
	}

}

bool RecgWeightedHoughForest(int *binNum, int *itemIdx, cv::Mat color, cv::Mat Sub, double *workPos, double *score, std::vector<int>& WHFClusterIdx ){
	//関数をvoid型からbool型に変更+ cv::Mar Subと WHF_cluster_idx を引数に追加 2015.04.26村田

		int bin,itemNum;
		int *ref_num;
		int refNum;

		cv::Mat testImage;
		//1チャンネルのグレーススケール画像を作成
		cv::cvtColor(color,testImage,CV_BGR2GRAY,1);
		bin=*binNum;
		itemNum=*itemIdx;

		//処理後書き換わってしまうので深いコピー
		cv::Mat Sub2 = Sub.clone();
		cv::Mat opening = Sub.clone();
		Detector detector;
		//入力画像のスケールをWHFcommon.hで定義したSCALEの値に変換(チャレンジ当日のスケールにあわせて調整)
		resize(testImage, testImage, Size() , SCALE, SCALE, INTER_CUBIC);
		resize(opening, opening, Size() , SCALE, SCALE, INTER_CUBIC);
		cv::erode(opening, opening, cv::Mat(), cv::Point(-1,-1), 2);//膨張収縮によるノイズ・穴の削減 村田
		cv::dilate(opening, opening, cv::Mat(), cv::Point(-1,-1), 4);
		//cv::erode(opening, opening, cv::Mat(), cv::Point(-1,-1), 1);
		//検出用関数
		detector.computeError(bin,itemNum,testImage,opening,workPos,score,ref_num);//リファレンス番号を返すように変更　村田


		//リファレンスの確認 ポインタ型です。村田
		std::cout<<"ref"<<*ref_num<<std::endl;;

		//LOG書き出しの判定
		#ifdef LOGMODE
		std::ofstream ofs( "./WHF_LOG.txt" ,std::ios::app);
		ofs<<"binNum="<<bin<<"\titemIdx="<<itemNum<<"\tworkPos="<<workPos[0]<<","<<workPos[1]<<"\tscore="<<*score<<"\tref_num="<<*ref_num<<std::endl;
		ofs.close();
		#endif
		
		cv::Mat WHFCluster( HEIGHT, WIDTH, CV_8UC1, cv::Scalar(0) );
		//セグメンテーション
		refNum = *ref_num;

		//入力部分書き換え(金子)
		WHFCluster = item_segmentation(Sub, (int)workPos[0], (int)workPos[1], itemNum, refNum);

		int	cnt;
		cnt =0;
		for( int i=0 ; i<WIDTH*HEIGHT ; i++ ){
			if(	Sub2.data[i] != 0 ){
				if( WHFCluster.data[i] != 0 ){
					WHFClusterIdx.push_back( cnt );
				}
				cnt++;
			}
		}

		cv::imwrite("function_test.bmp", WHFCluster);


		return true;
}

bool Recg( int **binIdx, int *binNum, int *itemIdx, double *pnt, unsigned char *depth, unsigned char *color, int *flag, 
		   double *workPos, unsigned char *cluster ){

	// 中心座標の初期化
	workPos[0] = -1;  //例外値を入れる
	workPos[1] = -1;  //認識できてたら更新される
	*flag = -1;


	FILE		*fp;
	char		funcname[256], name[256];
	double		deg2rad, rad2deg;

	deg2rad = M_PI/180.0;
	rad2deg = 180.0/M_PI;

	strcpy( funcname,"Recg()" );
	if ((fp = fopen("log_Recg.txt", "w")) == NULL) {
		fprintf( SE, "!!Error!! %s\n", funcname );
		fprintf( SE, "  ログファイルの書き出しができません．\n");
		return false;
	}


	// Bin の情報の表示
	fprintf( fp,"binIdx:\n");
	for( int j=0 ; j<12 ; j++ ){
		fprintf( fp,"%2d: ", j );
		for( int i=0 ; i<10 ; i++ ){
			fprintf( fp,"%d ", binIdx[j][i] );
		}
		fprintf( fp,"\n" );
	}
	fprintf( fp,"binNum: %d\n", *binNum );
	fprintf( fp,"itemIdx: %d\n", *itemIdx );
	fprintf( fp,"flag: %d\n", *flag );

	//=========================================================================//
	// 前処理．
	// ビンの内部のデータの取り出し，RGB情報付き点群データの生成．
	//=========================================================================//


	// depth画像の生成
	cv::Mat d_img;
	d_img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	for( int i=0 ; i<WIDTH*HEIGHT; i++ ){
		d_img.at<uchar>( i / WIDTH, i % WIDTH ) = depth[i];
	}


	cv::Mat median_img;
	cv::medianBlur( d_img, median_img, 3 );
	for( int i=0 ; i<4 ; i++ ){
		cv::medianBlur( median_img, median_img, 3 );
	}

	cv::Mat im_label_mask;
	im_label_mask = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	SegmentationNoiseReduction(median_img, im_label_mask );



	// データ変換．未計測点を除いた点のインデクスはPntIdxに入っている．
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> PntIdx;
	if(seg_noise_reduction == 1){
		Pnt2PCDXYZ( pnt, im_label_mask, WIDTH*HEIGHT, cloud, PntIdx );
	}else{
		Pnt2PCDXYZ( pnt, median_img, WIDTH*HEIGHT, cloud, PntIdx );
	}
	// ビンのサイズパラメータの読み込み．
	//FILE				*fp_bin;
	//char				dum[256];
	//char				bin_param_fname[FNL];
	//errno_t				error_check;
	//fprintf( fp,"Loading bin parameter\n" );
	//sprintf( bin_param_fname,"%s", BIN_PARAM_NAME );
	//if( (error_check = fopen_s( &fp_bin, bin_param_fname, "r" )) != 0 ){
	//	fprintf( fp,"!!Error %s cannot be load.\n", bin_param_fname );
	//	return false;
	//};

	//20150429-秋月追記↓↓↓
	// ビン内のデータの取り出し．
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sub (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> SubIdx;
	//BinRemove( cloud, dist_cam2bin[*binNum], cam_inclination, bin_width[*binNum], cloud_sub, SubIdx );
	//20150429-秋月追記↑↑↑
	
	//20150518-秋月追記↓↓↓
	//ビン横幅だけを使ってデータを除去
	BinRemove2( cloud, bin_width[*binNum], cloud_sub, SubIdx );
	//BinAndNoiseRemove( cloud, bin_width[*binNum], d_img, cloud_sub, SubIdx );
	//20150518-秋月追記↑↑↑


	// 背景差分画像の作成．
	cv::Mat Sub_img; //
	 Sub_img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	for( int i=0 ; i<SubIdx.size() ; i++ ){
		unsigned char tmp;
		tmp = depth[ PntIdx[SubIdx[i]] ];
		 Sub_img.at<uchar>( PntIdx[SubIdx[i]] / WIDTH, PntIdx[SubIdx[i]] % WIDTH ) = tmp;
	}

	// 20150521 秋月追記．start
	//画像構造体
	cv::Mat	im_orgMedian;  //平滑化した入力画像
	cv::Mat im_edgeFlag;   //エッジフラグ

	//パラメータ
	int m_nPrmCannyEdge;
	int m_nPrmMedFilter;

	m_nPrmCannyEdge		=	80;		// キャニーエッジ
	m_nPrmMedFilter		=	9;		// メディアンフィルタ
	// (004) 入力距離画像を処理用メモリへコピー
	//im_orgMedian	=	Sub_img.clone();
	im_orgMedian	=	median_img.clone();
	// (005) 処理用画像に２回メディアンフィルタかけて平滑化
	for(int i = 0; i < 2; i++)
	{
		medianBlur(im_orgMedian, im_orgMedian, m_nPrmMedFilter);
	}

#if OUTPUT3
	cv::imshow( "im_orgMedian  平滑化", im_orgMedian );
	cv::imwrite( "segclass_im_orgMedian.bmp", im_orgMedian );
	cv::waitKey(0);
#endif

	// (014) エッジ抽出(Canny) 
	cv::Mat im_tmp1;
	cv::Canny( im_orgMedian, im_tmp1, 0, m_nPrmCannyEdge, 3);

#if OUTPUT3
	cv::imshow( "im_tmp エッジ抽出", im_tmp1 );
	cv::imwrite( "segclass_edge.bmp", im_tmp1 );
	cv::waitKey(0);
#endif

	// (015) エッジ膨張 im_tmp1 -> im_tmp1
	cv::Mat C	=	cv::Mat::ones(3, 3, CV_32FC1);
	cv::dilate(im_tmp1, im_edgeFlag, C);

#if OUTPUT3
	cv::imshow( "im_edgeFlag エッジ膨張", im_edgeFlag );
	cv::imwrite( "segclass_dilated_edge.bmp", im_edgeFlag );
	cv::waitKey(0);
#endif


	// データ変換．未計測点を除いた点のインデクスはPntIdxに入っている．
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> PntEdgeIdx;
	Pnt2PCDXYZ_Edge( pnt, im_edgeFlag, WIDTH*HEIGHT, cloud_edge, PntEdgeIdx );
	//Pnt2PCDXYZ( pnt, im_tmp1, WIDTH*HEIGHT, cloud_edge, PntEdgeIdx );
#if OUTPUT2
	if( CheckSavePCD( cloud_edge ) == true ){
		pcl::io::savePCDFile( "cloud_edge.pcd", *cloud_edge ); 
	}
#endif
	// 20150521 秋月追記．end

	
	// シーン画像の作成（RGB） //WHF・FASTに入力するためこちらに移動　15.5.10村田
	cv::Mat color_img; // シーン画像（RGB）
	color_img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
	cv::Vec3b tmp;
	for( int i=0 ; i<WIDTH*HEIGHT ; i++ ){
		tmp(0) = color[ 3*i ];
		tmp(1) = color[ 3*i+1 ];
		tmp(2) = color[ 3*i+2 ];
		color_img.at<cv::Vec3b>( i / WIDTH, i % WIDTH ) = tmp;
	}

	// マスクした入力画像の作成．
	Mat masked_img( HEIGHT, WIDTH, CV_8UC3 );
	for( int i=0 ; i<SubIdx.size() ; i++ ){
		tmp(0) = color[ 3*PntIdx[SubIdx[i]] ];
		tmp(1) = color[ (3*PntIdx[SubIdx[i]])+1 ];
		tmp(2) = color[ (3*PntIdx[SubIdx[i]])+2 ];
		masked_img.at<cv::Vec3b>( PntIdx[SubIdx[i]] / WIDTH, PntIdx[SubIdx[i]] % WIDTH ) = tmp;
	}




	// Simple アルゴリズム用の処理
	// RGB情報つき点群の生成
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_subRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_subRGB->width = cloud_sub->width;
	cloud_subRGB->height = cloud_sub->height;
	cloud_subRGB->points.resize( cloud_sub->points.size() );
	for( int i=0 ; i<cloud_sub->points.size() ; i++ ){
		cloud_subRGB->points[i].x = cloud_sub->points[i].x;
		cloud_subRGB->points[i].y = cloud_sub->points[i].y;
		cloud_subRGB->points[i].z = cloud_sub->points[i].z;
		cloud_subRGB->points[i].r = color[ 3*PntIdx[SubIdx[i]]+2 ];
		cloud_subRGB->points[i].g = color[ 3*PntIdx[SubIdx[i]]+1 ];
		cloud_subRGB->points[i].b = color[ 3*PntIdx[SubIdx[i]] ];
	}


	// それぞれのアルゴリズムでの認識
	// すべて動作させて，最もいいものを出力させる．
	// シンプルアルゴリズムをケースごとに初期化 2015.05.06 武井
	double				SimpleScore, SimpleScore2_1, SimpleScore2_2, SimpleScore2_3;
	double				VPMScore, VPMScore2;
	double				WHFScore, WHFScore2;
	double				FASTScore2;
	double				SimpleworkPos[12];
	double				VPMworkPos[12];
	double				WHFworkPos[12];
	double              FASTworkPos[12];
	int					SimpleCenterIdx1, SimpleCenterIdx2, SimpleCenterIdx3;
	int					VPMCenterIdx;
	std::vector<int>	SimpleClusterIdx1, SimpleClusterIdx2, SimpleClusterIdx3;


	int					Error;
	double				Score;
	std::vector<int>	ClusterIdx;

	Error = 1;
	Score = DBL_MAX;

	int					SimpleError1, SimpleError2, SimpleError3, FASTError; //エラーのフラグ
	SimpleError1 = SimpleError2 = SimpleError3 = 1; //0:エラーなし．1:エラーあり．
	FASTError = 1; //この変数はdefaultで1にしておく．認識できた場合に0になります．

	// スコア，姿勢データの初期化
	SimpleScore = VPMScore = WHFScore = 0.0;
	SimpleScore2_1 = SimpleScore2_2 = SimpleScore2_3 = VPMScore2 = WHFScore2 = FASTScore2 = DBL_MAX;
	for( int i=0 ; i<12 ; i++ ){
		SimpleworkPos[i] = 0.0;
		VPMworkPos[i] = 0.0;
		WHFworkPos[i] = 0.0;
		FASTworkPos[i] = 0.0;
	}
	VPMworkPos[0] = VPMworkPos[4] = VPMworkPos[8] = 1.0;

	// ビンの種類を解釈 2015.05.06 武井
	int num_of_seg ;
	num_of_seg = 0 ;
	for( int i=0 ; i<10 ; i++ ) {
		//fprintf( stderr, "binIdx= %d\n", binIdx[i] ) ;
		if( 0<binIdx[*binNum][i] && binIdx[*binNum][i]<26 )	num_of_seg++ ;
	}

	//20150518-秋月追記↓↓↓
	// 0->single bin,  1->double bin,  2->multi bin
	int bin_condition;
	if( num_of_seg == 1 ){
		bin_condition = 0;
	}else if( num_of_seg == 2 ){
		bin_condition = 1;
	}else{
		bin_condition = 2;
	}
	//20150518-秋月追記↑↑↑

	//=========================================================================//
	// 認識処理
	//=========================================================================//
	
	//20150518-秋月追記↓↓↓

	//=========================================================================//
	// Cascated FAST がうまくいきそうな物体に関しては優先的に動かす
	//=========================================================================//
	if( C_FAST_SW[*itemIdx] == ON ){
		// Cascaded Fast
		if(RecgCascadedFast(itemIdx, color_img, Sub_img, ClusterIdx, FASTworkPos )==false){
			fprintf( fp,"!!Error has been occured in RecgCascadedFast.\n" );
			fprintf( fp,"  このエラーはFASTで認識できなかった場合も含みます.\n" );
			FASTError = 1;
		}else{
			fprintf( fp,"Cascaded Fast End\n" );
			FASTError = 0;
			workPos[0] = FASTworkPos[0];
			workPos[1] = FASTworkPos[1];
			*flag = METHOD_FAST;
		}
	}
	//=========================================================================//
	// Cascated FAST end
	//=========================================================================//



	//=========================================================================//
	// WHF or Simple or VPM によるマッチング start
	//=========================================================================//
	if( FASTError ){ //FASTで認識できたかどうかチェック．
		if( method[*itemIdx][bin_condition] == METHOD_SIMPLE ){
			// Simple セグメンテーションによるマッチング
			*flag = METHOD_SIMPLE;
			fprintf( fp,"RecgSimple Start1\n" );
			if( RecgSimple( num_of_seg, itemIdx, cloud_subRGB, SimpleworkPos, &SimpleScore, &SimpleCenterIdx1, SimpleClusterIdx1 ) == false ){
				fprintf( fp,"!!Error has been occured in RecgSimple().\n" );
				SimpleScore  = 0.0;
				SimpleError1 = 1;
			}else{
				SimpleError1 = 0;
			}
			fprintf( fp,"RecgSimple End1\n" );
			num_of_seg += 1 ;
			fprintf( fp,"RecgSimple Start2\n" );
			if( RecgSimple( num_of_seg, itemIdx, cloud_subRGB, SimpleworkPos, &SimpleScore, &SimpleCenterIdx2, SimpleClusterIdx2 ) == false ){
				fprintf( fp,"!!Error has been occured in RecgSimple().\n" );
				SimpleScore  = 0.0;
				SimpleError2 = 1;
			}else{
				SimpleError2 = 0;
			}
			fprintf( fp,"RecgSimple End2\n" );
			num_of_seg += 1 ;
			fprintf( fp,"RecgSimple Start3\n" );
			if( RecgSimple( num_of_seg, itemIdx, cloud_subRGB, SimpleworkPos, &SimpleScore, &SimpleCenterIdx3, SimpleClusterIdx3 ) == false ){
				fprintf( fp,"!!Error has been occured in RecgSimple().\n" );
				SimpleScore  = 0.0;
				SimpleError3 = 1;
			}else{
				SimpleError3 = 0;
			}
			fprintf( fp,"RecgSimple End3\n" );

			// 3つの中でいいものを選ぶためにスコアを算出
			rts_Score(itemIdx,color, PntIdx, SubIdx, SimpleClusterIdx1, &SimpleScore2_1);
			rts_Score(itemIdx,color, PntIdx, SubIdx, SimpleClusterIdx2, &SimpleScore2_2);
			rts_Score(itemIdx,color, PntIdx, SubIdx, SimpleClusterIdx3, &SimpleScore2_3);
			fprintf( fp,"SimpleScore2_1: %lf\n", SimpleScore2_1 );
			fprintf( fp,"SimpleScore2_2: %lf\n", SimpleScore2_2 );
			fprintf( fp,"SimpleScore2_3: %lf\n", SimpleScore2_3 );

			if( (SimpleScore2_2 <= SimpleScore2_1) && (SimpleScore2_3 <= SimpleScore2_1) ){ //Simpleケース1のスコアが最大
				*flag = METHOD_SIMPLE;
				//クラスタ内点群のコピー
				if( SimpleError1 == 0 ){
					workPos[0] = PntIdx[SubIdx[SimpleCenterIdx1]] % WIDTH;
					workPos[1] = PntIdx[SubIdx[SimpleCenterIdx1]] / WIDTH;
					for( int i=0 ; i<SimpleClusterIdx1.size() ; i++ ){
						ClusterIdx.push_back( SimpleClusterIdx1[i] );
					}
				}
				Error = 0;
				fprintf( fp,"Cluster = Simple1\n" );
			}else if( (SimpleScore2_1 < SimpleScore2_2) && (SimpleScore2_3 <= SimpleScore2_2) ){ //Simpleケース2のスコアが最大
				*flag = METHOD_SIMPLE;
				//クラスタ内点群のコピー
				if( SimpleError2 == 0 ){
					workPos[0] = PntIdx[SubIdx[SimpleCenterIdx2]] % WIDTH;
					workPos[1] = PntIdx[SubIdx[SimpleCenterIdx2]] / WIDTH;
					for( int i=0 ; i<SimpleClusterIdx2.size() ; i++ ){
						ClusterIdx.push_back( SimpleClusterIdx2[i] );
					}
				}
				Error = 0;
				fprintf( fp,"Cluster = Simple2\n" );
			}else{ //Simpleケース3のスコアが最大
				*flag = METHOD_SIMPLE;
				//クラスタ内点群のコピー
				if( SimpleError3 == 0 ){
					workPos[0] = PntIdx[SubIdx[SimpleCenterIdx3]] % WIDTH;
					workPos[1] = PntIdx[SubIdx[SimpleCenterIdx3]] / WIDTH;
					for( int i=0 ; i<SimpleClusterIdx3.size() ; i++ ){
						ClusterIdx.push_back( SimpleClusterIdx3[i] );
					}
				}
				Error = 0;
				fprintf( fp,"Cluster = Simple3\n" );
			}

		}else if( method[*itemIdx][bin_condition] == METHOD_WHF ){
			// Weighted Hough Forest によるマッチング
			*flag = METHOD_WHF;
			fprintf( fp,"RecgWeightedHoughForest Start\n" );
			if( RecgWeightedHoughForest( binNum, itemIdx, masked_img,Sub_img, workPos, &Score, ClusterIdx ) == false ){
				fprintf( fp,"!!Error has been occured in RecgWeightedHoughForest.\n" );
				Score = 0.0;
				Error = 1;
			}else{
				Error = 0;
			}
			fprintf( fp,"RecgWeightedHoughForest End\n" );
		}else{
			// VPM によるマッチング
			fprintf( fp,"RecgVPM Start\n" );
			*flag = METHOD_VPM;
			if( RecgVPM( itemIdx, cloud_sub, cloud_edge, workPos, &Score, &VPMCenterIdx, ClusterIdx ) == false ){
				fprintf( fp,"!!Error has been occured in RecgVPM().\n" );
				Score = 0.0;
				Error = 1;
			}else{
				workPos[0] = PntIdx[SubIdx[VPMCenterIdx]] % WIDTH;
				workPos[1] = PntIdx[SubIdx[VPMCenterIdx]] / WIDTH;
				Error = 0;
			}
			fprintf( fp,"RecgVPM End\n" );
		}
	}
	//=========================================================================//
	// WHF or Simple or VPM によるマッチング end
	//=========================================================================//




	//=========================================================================//
	// クラスタデータのコピー　start
	//=========================================================================//
	fprintf( fp,"Make cluster...\n" );
	if( (Error == 0) || (FASTError==0) ){
		for( int i=0 ; i<ClusterIdx.size() ; i++ ){
			cluster[ PntIdx[SubIdx[ClusterIdx[i]]] ] = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
		}
		//*flag = 1;
	}else{ // エラー処理．距離データをそのままコピー
		for( int i=0 ; i<WIDTH*HEIGHT ; i++ ){
			cluster[ i ] = depth[ i ];
		}
		fprintf( fp,"Cluster = No.\n" );
		*flag = METHOD_ERROR;
	}
	fprintf( fp,"Make cluster...End\n" );
	//=========================================================================//
	// クラスタデータのコピー　end
	//=========================================================================//

	fprintf( SE,"Error         = %d\n", Error );
	fprintf( SE,"Simple Error1 = %d\n", SimpleError1 );
	fprintf( SE,"Simple Error2 = %d\n", SimpleError2 );
	fprintf( SE,"Simple Error3 = %d\n", SimpleError3 );
	fprintf( SE,"FASTError     = %d\n", FASTError );


#if OUTPUT3
	//=========================================================================//
	// デバッグデータの出力
	//=========================================================================//

	// クラスタ画像の作成．
	int imgX, imgY;
	if( (method[*itemIdx][bin_condition] == METHOD_VPM) && (Error == 0) ){
		cv::Mat cluster_img_vpm; // VPM
		cluster_img_vpm = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		for( int i=0 ; i<ClusterIdx.size() ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			tmp(1) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			tmp(2) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			cluster_img_vpm.at<cv::Vec3b>( PntIdx[SubIdx[ClusterIdx[i]]] / WIDTH, PntIdx[SubIdx[ClusterIdx[i]]] % WIDTH ) = tmp;
		}
		imgX = PntIdx[SubIdx[VPMCenterIdx]] % WIDTH;
		imgY = PntIdx[SubIdx[VPMCenterIdx]] / WIDTH;
		//cv::circle( cluster_img_vpm, cv::Point(imgX, imgY), 6, cv::Scalar(0,0,255), -1, CV_AA, 0 );
		//cv::resize( cluster_img_vpm, cluster_img_vpm, cv::Size() , SCALE, SCALE, INTER_CUBIC);

		cv::imwrite( "cluster_img_vpm.bmp", cluster_img_vpm );
	}

	// シンプルアルゴリズムをケースごとに保存
	if( (method[*itemIdx][bin_condition] == METHOD_SIMPLE) && (SimpleError1 == 0) ){
		cv::Mat cluster_img_simple; // Simple
		cluster_img_simple = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		for( int i=0 ; i<SimpleClusterIdx1.size() ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = depth[ PntIdx[SubIdx[SimpleClusterIdx1[i]]] ];
			tmp(1) = depth[ PntIdx[SubIdx[SimpleClusterIdx1[i]]] ];
			tmp(2) = depth[ PntIdx[SubIdx[SimpleClusterIdx1[i]]] ];
			cluster_img_simple.at<cv::Vec3b>( PntIdx[SubIdx[SimpleClusterIdx1[i]]] / WIDTH, PntIdx[SubIdx[SimpleClusterIdx1[i]]] % WIDTH ) = tmp;
		}
		imgX = PntIdx[SubIdx[SimpleCenterIdx1]] % WIDTH;
		imgY = PntIdx[SubIdx[SimpleCenterIdx1]] / WIDTH;
		//cv::circle( cluster_img_simple, cv::Point(imgX, imgY), 6, cv::Scalar(0,0,255), -1, CV_AA, 0 );
		//cv::resize( cluster_img_simple, cluster_img_simple, cv::Size() , SCALE, SCALE, INTER_CUBIC);
		cv::imwrite( "cluster_img_simple1.bmp", cluster_img_simple );
	}

	if( (method[*itemIdx][bin_condition] == METHOD_SIMPLE) && (SimpleError2 == 0) ){
		cv::Mat cluster_img_simple; // Simple
		cluster_img_simple = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		for( int i=0 ; i<SimpleClusterIdx2.size() ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = depth[ PntIdx[SubIdx[SimpleClusterIdx2[i]]] ];
			tmp(1) = depth[ PntIdx[SubIdx[SimpleClusterIdx2[i]]] ];
			tmp(2) = depth[ PntIdx[SubIdx[SimpleClusterIdx2[i]]] ];
			cluster_img_simple.at<cv::Vec3b>( PntIdx[SubIdx[SimpleClusterIdx2[i]]] / WIDTH, PntIdx[SubIdx[SimpleClusterIdx2[i]]] % WIDTH ) = tmp;
		}
		imgX = PntIdx[SubIdx[SimpleCenterIdx2]] % WIDTH;
		imgY = PntIdx[SubIdx[SimpleCenterIdx2]] / WIDTH;
		//cv::circle( cluster_img_simple, cv::Point(imgX, imgY), 6, cv::Scalar(0,0,255), -1, CV_AA, 0 );
		//cv::resize( cluster_img_simple, cluster_img_simple, cv::Size() , SCALE, SCALE, INTER_CUBIC);
		cv::imwrite( "cluster_img_simple2.bmp", cluster_img_simple );
	}

	if( (method[*itemIdx][bin_condition] == METHOD_SIMPLE) && (SimpleError3 == 0) ){
		cv::Mat cluster_img_simple; // Simple
		cluster_img_simple = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		for( int i=0 ; i<SimpleClusterIdx3.size() ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = depth[ PntIdx[SubIdx[SimpleClusterIdx3[i]]] ];
			tmp(1) = depth[ PntIdx[SubIdx[SimpleClusterIdx3[i]]] ];
			tmp(2) = depth[ PntIdx[SubIdx[SimpleClusterIdx3[i]]] ];
			cluster_img_simple.at<cv::Vec3b>( PntIdx[SubIdx[SimpleClusterIdx3[i]]] / WIDTH, PntIdx[SubIdx[SimpleClusterIdx3[i]]] % WIDTH ) = tmp;
		}
		imgX = PntIdx[SubIdx[SimpleCenterIdx3]] % WIDTH;
		imgY = PntIdx[SubIdx[SimpleCenterIdx3]] / WIDTH;
		//cv::circle( cluster_img_simple, cv::Point(imgX, imgY), 6, cv::Scalar(0,0,255), -1, CV_AA, 0 );
		//cv::resize( cluster_img_simple, cluster_img_simple, cv::Size() , SCALE, SCALE, INTER_CUBIC);
		cv::imwrite( "cluster_img_simple3.bmp", cluster_img_simple );
	}


	if( (method[*itemIdx][bin_condition] == METHOD_WHF) && (Error == 0) ){
		cv::Mat cluster_img_whf; // WHF
		cluster_img_whf = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		for( int i=0 ; i<ClusterIdx.size() ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			tmp(1) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			tmp(2) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			cluster_img_whf.at<cv::Vec3b>( PntIdx[SubIdx[ClusterIdx[i]]] / WIDTH, PntIdx[SubIdx[ClusterIdx[i]]] % WIDTH ) = tmp;
		}
		imgX = WHFworkPos[0];
		imgY = WHFworkPos[1];
		//cv::circle( cluster_img_whf, cv::Point(imgX, imgY), 6, cv::Scalar(0,0,255), -1, CV_AA, 0 );
		//cv::resize( cluster_img_whf, cluster_img_whf, cv::Size() , SCALE, SCALE, INTER_CUBIC);
		cv::imwrite( "cluster_img_whf.bmp", cluster_img_whf );
	}
	if( (C_FAST_SW[*itemIdx]) && (FASTError == 0) ){
		cv::Mat cluster_img_fast; // FAST
		cluster_img_fast = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		for( int i=0 ; i<ClusterIdx.size() ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			tmp(1) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			tmp(2) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			cluster_img_fast.at<cv::Vec3b>( PntIdx[SubIdx[ClusterIdx[i]]] / WIDTH, PntIdx[SubIdx[ClusterIdx[i]]] % WIDTH ) = tmp;
		}
		//imgX = WHFworkPos[0];
		//imgY = WHFworkPos[1];
		//cv::circle( cluster_img_whf, cv::Point(imgX, imgY), 6, cv::Scalar(0,0,255), -1, CV_AA, 0 );
		//cv::resize( cluster_img_fast, cluster_img_fast, cv::Size() , SCALE, SCALE, INTER_CUBIC);
		cv::imwrite( "cluster_img_fast.bmp", cluster_img_fast );
	}

	cv::resize( masked_img, masked_img, cv::Size() , SCALE, SCALE, INTER_CUBIC);
	cv::imwrite( "sub_img.bmp", masked_img ); // 差分画像（RGB）
	//cv::imwrite( "color_img.bmp", color_img ); // シーン画像（RGB）

	cv::Mat bg_img; // 背景差分後の画像
	bg_img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	for( int i=0 ; i<SubIdx.size() ; i++ ){
		unsigned char tmp;
		tmp = depth[ PntIdx[SubIdx[i]] ];
		bg_img.at<uchar>( PntIdx[SubIdx[i]] / WIDTH, PntIdx[SubIdx[i]] % WIDTH ) = tmp;
	}
	cv::resize( bg_img, bg_img, cv::Size() , SCALE, SCALE, INTER_CUBIC);
	cv::imwrite( "bg_img.bmp", bg_img );
	
	cv::resize( d_img, d_img, cv::Size() , SCALE, SCALE, INTER_CUBIC);
	cv::imwrite( "depth_img.bmp", d_img );
	//cv::imwrite( "depth_median_img.bmp", median_img );
#endif

#if OUTPUT2
	pcl::io::savePCDFileASCII( "cloud_subRGB.pcd", *cloud_subRGB );
	pcl::io::savePCDFileASCII( "cloud_input.pcd", *cloud );	
#endif
	fclose( fp );

	return true;

}

