///////////////////////////////////////////////////////////////////////////
//
//  Label Optimization
//
//  Shuichi AKIZUKI
//
//  (C) 2016 ISL, Chukyo University All rights reserved.
//
//  Note:
//
//  2016.03.29
//  Rev.A
//  2016.04.15
//	Score_UAを改良して処理時間を高速化．
//  画像をリサイズして，処理を高速化．本番用プログラムに実装するときは，
//  把持点等の位置もリサイズしたことに気をつけること．
//	リサイズを実施した部分には「RESIZE」と書いておいた．
//	2016.04.24
//  main2.cppからは，
//  [Segmentation] -> [GFE] -> [CNN]  
//  の結果を読み込んで，実験をおこなうためのソースファイルとする．
//  仮説データのフォーマットが変わったことに注意すること．
//  2016.04.26
//  main.cppで直した部分のコードを持ってきた．
//  2016.05.18
//  統合用に関数化を進めた．PCLはいらないので，抜く．
//  2016.05.18
//  LabelOptimizationを関数recgAPC2016()に作り変えた．
//  ASJ実装時に
//  #include "use_OpenCV.h" をコメントアウト
//
//  
//  2016.06.23
//  変更点1.
//  nTargetItemIdx = 0 のときの処理を追加．
//  この場合は最も信頼度の高い物体IDにすべてのセグメントを書き換えて出力
//  
//
///////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include "common.h"
#include "Labeling.h"
//#include "use_OpenCV.h"
#include "ColorTableRGB2.h"
#include "GA.h"
#include "LabelOptimization.h"
//飯塚セグメンテーション
#include "color_segmentation.h"
#include "../Chubu/run_cascadedFAST.h"

//CNN関連のインクルード
#include "../Chubu/run_cnn.h"

#include "../FGE/run_FGE.h"



//仮置き台での認識のテスト用（強制的にnTargetItemIdx = 0にします）
//#define DBG_KARIOKI_TEST
//特定の画像を読み込むためのフラグ
//#define DBG_IMG_LOAD



//ここにcommonヘッダに書きたい内容を記述（データディレクトリなど）
#define PARAM_DIR		"C:\\Temp\\RecgAPC2016Param\\"		/* パラメータディレクトリ名 */
#define PARAM_NAME		"param.txt"							/* パラメータファイル名 */

void calcNormalMap_recg( cv::Mat *InImg, int PCASize, int ResizeRate, int BackGroundID, unsigned char range_th, float noise_theta_th, bool log_img, cv::Mat *top, cv::Mat *front, cv::Mat *side );


//20160519 ソート用
bool CombSort( const std::vector<double>& values, std::vector<int>& Idx ){

	static int	n, Ndata;
	int			gap, swap_int;
	int			tmp;

	Ndata = values.size();
	if( Ndata != Idx.size() ){
		fprintf( stderr,"Error! Size of vp and Idx is different.\n" );
		fprintf( stderr,"		Size of values : %d\n", values.size() );
		fprintf( stderr,"		Size of Idx: %d\n", Idx.size() );
		return false;
	}

	gap = Ndata;
	swap_int = 1;
	while( gap>1 || swap_int ) {
		gap = (int)((double)gap/1.3) ;
		if(gap == 9 || gap == 10) gap = 11 ;
		swap_int = 0;
		for( n=0; n<(Ndata-gap) ; n++ ){
			if( values[Idx[n]] < values[Idx[n+gap]] ){
				tmp = Idx[n]; Idx[n] = Idx[n+gap]; Idx[n+gap] = tmp;
				swap_int = 1;
			}
		}
	}

	return true;
}


//bool RecgAPC2016( int **binIdx, int *binNum, int *itemIdx, int orderBin, double *pnt, unsigned char *depth, unsigned char *color, int backgroundID,
//				 std::vector<int>& work_i,  std::vector<int>& work_j,  std::vector<double>& work_score, unsigned char *cluster, 
//				 std::vector<int>& nt_itemIdx, std::vector<int>& nt_i, std::vector<int>& nt_j, std::vector<double>& nt_score, unsigned char *nt_cluster,
//				 unsigned char *c_map, unsigned char *id_map ){
bool RecgAPC2016( int *nItemIndices, int nItemNum, int nTargetItemIdx, int nTargetBinIdx, double *pnt, unsigned char *depth, unsigned char *color, int backgroundID,
				 std::vector<int>& work_i,  std::vector<int>& work_j,  std::vector<double>& work_score, unsigned char *cluster, 
				 std::vector<int>& nt_itemIdx, std::vector<int>& nt_i, std::vector<int>& nt_j, std::vector<double>& nt_score, unsigned char *nt_cluster,
				 unsigned char *c_map, unsigned char *id_map ){

//仮置き台での認識ようオプション．本番ではOFFにすること					 
#ifdef DBG_KARIOKI_TEST
	nTargetItemIdx = 0;
#endif

	char		im_color_fname[FNL], im_depth_fname[FNL], pnt_fname[FNL], paramfname[FNL], hyp_fname[FNL], fast_hyp_fname[FNL];
	char		artfname[FNL];
	double		time_func_start, time_func_end, time1, time2;

	time_func_start = static_cast<double>(cv::getTickCount());
	fprintf( SE,"RecgAPC2016 start\n" );
	//parameter of GA
	int			population; 
	int			n_on;	// # of ON bit of initial individuals
	int			n_gen;  // # of max generation
	int			n_stable_gen;  // limit of stable generation

	// 実行時オプション類
	bool		use_artifitial, use_viewer, log, log2, log_img, log_img2;

	//bin parameter
	int			n_item;
	int			viewpoint; //backgroundIDから判断する視点情報．
						   //backgroundIDから入るのが奇数なら斜め，偶数なら正面．

	//各アルゴリズム使用に関するON/OFF制御フラグ
	bool		use_CNN_DATA, use_FAST_DATA;
	bool		use_CNN, use_FAST;
	bool		use_SP_SEG, use_TEX_SEG, use_NORMAL_SEG;
	int			segMethod; //0.デプス 1.デプス＋テクスチャ 2.法線方向
	bool		use_SegCenter;

	//画像処理パラメータ
	int			n_closing_cluster;	//出力のクラスタデータに掛けるクロージングの回数
	int			min_segment_size;   //出力する最終セグメントサイズ
	int			max_grasp_point;   //CNNで識別する把持点の最大数
	int			min_grasp_point;   //CNNで識別する把持点の最小数
	int			grasp_step;			//把持点のステップ数

	// Default parameters.
	strcpy( im_color_fname,"color.bmp" );
	strcpy( im_depth_fname,"depth.bmp.bmp" );
	strcpy( pnt_fname,"point3d.pnt" );
	strcpy( hyp_fname,"hyp.txt" );
	strcpy( paramfname,"param.txt" );
	strcpy( artfname,"art_gene.txt" );

	population = 100;
	n_on = 5;
	n_gen = 1000;
	n_stable_gen = 100;
	use_artifitial = false;
	use_viewer = false;
	use_CNN_DATA = use_FAST_DATA = false;
	use_SP_SEG = use_TEX_SEG = use_NORMAL_SEG = false;
	use_CNN = use_FAST = false;
	log = log2 = log_img = log_img2 = false;
	use_SegCenter = false;
	n_item = nItemNum;
	n_closing_cluster = 2;
	min_segment_size = 2000;
	max_grasp_point = 20;
	min_grasp_point = 3;
	grasp_step = 50000;
	segMethod = 0;
	//偶数だったら斜め，奇数なら正面
	// -> 斜めならviewpoint = 1, 正面ならviewpoint = 2 
	if( backgroundID%2 ) viewpoint = 2;
	else				 viewpoint = 1;


	FILE		*fp;
	char		funcname[256];
	double		deg2rad, rad2deg;

	deg2rad = M_PI/180.0;
	rad2deg = 180.0/M_PI;

	//strcpy( funcname,"Recg()" );
	if ((fp = fopen("log_Recg.txt", "w")) == NULL) {
		fprintf( SE, "!!Error!! %s\n", funcname );
		fprintf( SE, "  ログファイルの書き出しができません．\n");
		return false;
	}


	// Bin の情報の表示
	fprintf( fp,"viewpoint:%d\n", viewpoint );
	fprintf( fp,"backgroundID:%d\n", backgroundID );
	fprintf( fp,"nItemNum:\n");
	for( int i=0 ; i<nItemNum ; i++ ){
		fprintf( fp,"%d ", nItemIndices[i] );
	}
	fprintf( fp,"\n" );
	fprintf( fp,"nItemNum: %d\n", nItemNum );
	fprintf( fp,"nTargetItemIdx: %d\n", nTargetItemIdx );
	fclose( fp );



	// [0100] データ入力
	// [0101] 距離画像生成
	cv::Mat d_img;
	d_img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	for( int i=0 ; i<WIDTH*HEIGHT; i++ ){
		d_img.at<uchar>( i / WIDTH, i % WIDTH ) = depth[i];
	}


	// [0102] RGB画像生成
	cv::Mat color_img; // シーン画像（RGB）
	color_img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
	cv::Vec3b tmp;
	for( int i=0 ; i<WIDTH*HEIGHT ; i++ ){
		tmp(0) = color[ 3*i ];
		tmp(1) = color[ 3*i+1 ];
		tmp(2) = color[ 3*i+2 ];
		color_img.at<cv::Vec3b>( i / WIDTH, i % WIDTH ) = tmp;
	}

//2016.06.24 秋月 画像を読み込むためのフラグ
//binの中に保存されるd_img.bmpとcolor_img.bmpで再現実験するため
#ifdef DBG_IMG_LOAD
	cv::Mat d_img2 = cv::imread( "C:\\Temp\\DBG_IMG\\d_img.bmp", 1 );
	cv::Mat c_img2 = cv::imread( "C:\\Temp\\DBG_IMG\\color_img.bmp", 1 );
	cv::Mat d_img22;
	cv::cvtColor( d_img2, d_img22, CV_RGB2GRAY );
	d_img = d_img22.clone();
	color_img = c_img2.clone();
#endif

	//読み込んだ画像の表示
	if( use_viewer ){
		cv::imshow( "d_img", d_img );
		cv::imshow( "color_img", color_img );
		cv::waitKey();
	}


	//[0103] パラメータファイル読み込み
	FILE	*fp_param;
	char	line[256];
	char	param_name[256];
	char	value[256];
	sprintf( paramfname,"%s%s", PARAM_DIR, PARAM_NAME );
	if( (fp_param = fopen( paramfname, "r" )) == NULL ){
		fprintf( SE,"!!Error!! Module:[0103]\n" );
		fprintf( SE,"  %s could not be read.\n", paramfname );
		return false;
	}else{
		while( fgets(line, 256, fp_param) != NULL){
			if( (line[0] == '#') || (!strcmp( line, "\n" )) ){
				//# The line starting '#' and 'Return' is skipped.
			}else{
				sscanf(line, "%s %s\n", &param_name, &value);  //Split 'line' into 'param_name' and 'value'

				if( !strcmp( param_name, "USE_ARTIFITIAL_GENE" ) ){
					use_artifitial = true;
					strcpy( artfname, value );
				}
				if( !strcmp( param_name, "USE_VIEWER" ) ){
					use_viewer = true;
				}

				// Parameter of GA
				if( !strcmp( param_name, "POPULATION" ) ) population = atoi( value );
				if( !strcmp( param_name, "N_ON" ) ) n_on = atoi( value );
				if( !strcmp( param_name, "N_GENERATION" ) ) n_gen = atoi( value );
				if( !strcmp( param_name, "N_STABLE_GENERATION" ) ) n_stable_gen = atoi( value );

				// Input files
				if( !strcmp( param_name, "IMG_COLOR_NAME" ) ) strcpy( im_color_fname, value );
				if( !strcmp( param_name, "IMG_DEPTH_NAME" ) ) strcpy( im_depth_fname, value );
				if( !strcmp( param_name, "PNT_NAME" ) ) strcpy( pnt_fname, value );
				if( !strcmp( param_name, "CNN_HYP_NAME" ) ) strcpy( hyp_fname, value );
				if( !strcmp( param_name, "FAST_HYP_NAME" ) ) strcpy( fast_hyp_fname, value );

				// アルゴリズムのON/OFF制御フラグ
				if( !strcmp( param_name, "USE_CNN_DATA" ) ) use_CNN_DATA = true;
				if( !strcmp( param_name, "USE_FAST_DATA" ) ) use_FAST_DATA = true;
				if( !strcmp( param_name, "USE_CNN" ) ) use_CNN = true;
				if( !strcmp( param_name, "USE_FAST" ) ) use_FAST = true;
				if( !strcmp( param_name, "USE_SP_SEG" ) ) use_SP_SEG = true;
				if( !strcmp( param_name, "USE_TEX_SEG" ) ){
					use_TEX_SEG = true;
					segMethod = 1;
				}
				if( !strcmp( param_name, "USE_NORMAL_SEG" ) ){
					use_NORMAL_SEG = true;
					segMethod = 2;
				}
				if( !strcmp( param_name, "USE_SEG_CENTER" ) ) use_SegCenter = true;
				if( !strcmp( param_name, "MAX_CNN_POINT" ) ) max_grasp_point = atoi( value );
				if( !strcmp( param_name, "MIN_CNN_POINT" ) ) min_grasp_point = atoi( value );

				if( !strcmp( param_name, "LOG" ) ) log = true;
				if( !strcmp( param_name, "LOG2" ) ) log2 = true;
				if( !strcmp( param_name, "LOG_IMG" ) ) log_img = true;
				if( !strcmp( param_name, "LOG_IMG2" ) ) log_img2 = true;

				//画像処理パラメータ
				if( !strcmp( param_name, "N_CLOSING" ) ) n_closing_cluster = atoi( value );
				if( !strcmp( param_name, "MIN_SEGMENT_SIZE" ) ) min_segment_size = atoi( value );
				if( !strcmp( param_name, "MAX_GRASP_POINT" ) ) max_grasp_point = atoi( value );
				if( !strcmp( param_name, "MIN_GRASP_POINT" ) ) min_grasp_point = atoi( value );
				if( !strcmp( param_name, "GRASP_STEP" ) ) grasp_step = atoi( value );
			}
		}
		fclose( fp_param );
	}

	fprintf( SE,"===============Parameter===============\n");
	if( use_artifitial )   fprintf( SE,"Use artifitial gene: %s\n", artfname );
	if( use_viewer )   fprintf( SE,"Use image viewer\n" );
	if( use_CNN_DATA )   fprintf( SE,"CNNのダミーデータを使います．\n" );
	if( use_FAST_DATA )   fprintf( SE,"FASTのダミーデータを使います．\n" );
	if( use_CNN )   fprintf( SE,"CNNが動作します．\n" );
	if( use_FAST )   fprintf( SE,"FASTが動作します．\n" );
	if( use_TEX_SEG )       fprintf( SE,"テクスチャセグメンテーションが動作します．\n" );
	else if( use_SP_SEG )   fprintf( SE,"Super pixelセグメンテーションが動作します．\n" );
	else if( use_NORMAL_SEG )   fprintf( SE,"法線方向ベースセグメンテーションが動作します．\n" );
	else                    fprintf( SE,"Depth セグメンテーションのみが動作します．\n" );
	if( log )   fprintf( SE,"Save log images\n" );

	if( use_SegCenter )   fprintf( SE,"セグメント重心を把持点とします．\n" );
	fprintf( SE," Max grasp point: %d\n", max_grasp_point );
	fprintf( SE," Min grasp point: %d\n", min_grasp_point );
	fprintf( SE," 識別点のステップ数: %d\n", grasp_step );

	fprintf( SE,"\n");
	fprintf( SE,"Genetic Algorithm\n");
	fprintf( SE," Population: %d\n", population );
	fprintf( SE," # of ON bit: %d\n", n_on );
	fprintf( SE," # of max generation: %d\n", n_gen );
	fprintf( SE," Limit of stable generation: %d\n", n_stable_gen );
	fprintf( SE,"\n");
	fprintf( SE,"Bin parameter\n");
	fprintf( SE," Number of items: %d\n", n_item );
	if( viewpoint == 1 )      fprintf( SE," 視点: 斜め\n" );
	else if( viewpoint == 2 ) fprintf( SE," 視点: 正面\n" );
	fprintf( SE,"Param Imaging\n");
	fprintf( SE," N closing %d\n", n_closing_cluster );
	fprintf( SE," Min Segment size%d\n", min_segment_size );
	fprintf( SE,"===============Parameter===============\n");

	//この関数のログファイル
	FILE *fp_logRecg;
	if( log ){
		if( (fp_logRecg = fopen( "log_recgAPC.txt", "w" )) == NULL ){
			fprintf( stderr,"File open error. recgAPC2016()\n" );
			return false;
		}
	}

	if( log_img ){
		cv::imwrite( "d_img.bmp", d_img );
		cv::imwrite( "color_img.bmp", color_img );
	}

	//[0103] 画像のサイズ変換
	// 距離画像が24bitだったのでグレースケールに変換
	if( log ){
		fprintf( fp_logRecg,"画像のサイズ変換\n" );
		fprintf( fp_logRecg,"Module:[0103]\n" );
	}
	cv::Mat im_depth3;
	cv::Mat im_depth; 
	//RESIZE 画像データのリサイズ
	cv::resize( d_img, im_depth, cv::Size(), SCALE, SCALE );

	// カラー画像の読み込み
	cv::Mat im_color;
	//RESIZE 画像データのリサイズ
	cv::resize( color_img, im_color, cv::Size(), SCALE, SCALE );


	//[0104] 仮説データの読み込み -> 2016.04.24新しい仮説仕様に変更
	//[0104a] CNNの結果の読み込み
	// hyp に姿勢仮説が入る．
	if( log ){
		fprintf( fp_logRecg,"CNN仮説データ読み込み\n" );
		fprintf( fp_logRecg,"Module:[0104]\n" );
	}
	FILE		*fp_hyp;
	int			n_hyp;
	std::vector<Hyp> hyp;

	if( use_CNN_DATA ){
		if( (fp_hyp = fopen( hyp_fname, "r" )) == NULL ){
			fprintf( SE,"!!Error!! Module:[0102a]\n" );
			fprintf( SE,"  %s could not be read.\n", hyp_fname );
			return false;
		}else{
			Hyp tmp_hyp;
			int hyp_id, grasp_x, grasp_y, item_id, hoge;
			while( fscanf( fp_hyp,"%d,%d,%d,%d,%d", &hyp_id, &grasp_x, &grasp_y, &item_id, &hoge ) != EOF ){
				tmp_hyp.grasp.x = grasp_x;
				tmp_hyp.grasp.y = grasp_y;
				tmp_hyp.label = item_id;
				tmp_hyp.score = DL_SCORE; //DeepLearningの信頼度（現状は決めうち）
				tmp_hyp.method = CNN_LABEL;

				hyp.push_back( tmp_hyp );
			}
		
		}
		n_hyp = hyp.size();
		fclose( fp_hyp );
	}else{
		if( log ){
			fprintf( fp_logRecg,"CNNの仮説データを読み込みません．\n" );
		}
	}


	//[0105] CascadedFASTの結果の読み込み
	if( log ){
		fprintf( fp_logRecg,"FAST仮説データ読み込み\n" );
		fprintf( fp_logRecg,"Module:[0105]\n" );
	}
	if( use_FAST_DATA ){
		FILE		*fp_fast;
		std::vector<int> fast_i;
		std::vector<int> fast_j;
		std::vector<int> fast_id;
		std::vector<int> fast_n_match;
		std::vector<double> fast_score;
		if( (fp_fast = fopen( fast_hyp_fname, "r" )) == NULL ){
			if( log ){ 
				fprintf( fp_logRecg,"!!Error!! Module:[0105]\n" );
				fprintf( fp_logRecg,"  %s could not be read.\n", fast_hyp_fname );
				fclose( fp_logRecg );
			}
			return false;
		}else{
			Hyp tmp_hyp;
			int i, j, id, n_match;
			double score;
			while( fscanf( fp_fast,"%d,%d,%d,%d,%lf", &i, &j, &id, &n_match, &score ) != EOF ){
				fast_i.push_back( i );
				fast_j.push_back( j );
				fast_id.push_back( id );
				fast_n_match.push_back( n_match );
				fast_score.push_back( score );
			}
		
		}
		fprintf( SE,"%d fast keypoints are read.\n", fast_i.size() );
		if( log ) fprintf( fp_logRecg,"%d fast keypoints are read.\n", fast_i.size() );
		fclose( fp_fast );
		debug_FAST_res( &im_depth3, fast_i, fast_j, fast_id, fast_n_match );
		//Cascaded FASTの結果を仮説データとして取り込む
		for( int i=0 ; i<fast_id.size() ; i++ ){
			Hyp tmp_hyp;
			tmp_hyp.grasp.x = fast_i[i];
			tmp_hyp.grasp.y = fast_j[i];
			tmp_hyp.label = fast_id[i];
			tmp_hyp.score = fast_score[i];
			tmp_hyp.method = FAST_LABEL;

			hyp.push_back( tmp_hyp );
		}
		n_hyp = hyp.size();
	}else{
		if( log ){
			fprintf( fp_logRecg,"FASTの仮説データを読み込みません．\n" );
		}
	}




	//--------------------------------------------------------------------------------//
	//[04 セグメンテーション]
	//--------------------------------------------------------------------------------//

	time1 = static_cast<double>(cv::getTickCount());
	double Seg_time;
	//[0402] セグメンテーション
	//[0402a] smoothing
	if( log ){
		fprintf( fp_logRecg,"セグメンテーション.\n");
		fprintf( fp_logRecg,"Module:[0402]\n" );
	}

	cv::Mat im_depth_smooth = im_depth.clone();
	for(int i = 0; i < 2; i++)
	{
	cv::medianBlur( im_depth_smooth, im_depth_smooth, 25 );
	}
	if( use_viewer ){
		cv::imshow( "im_depth_smooth", im_depth_smooth );
		cv::waitKey();
	}

	//--------------------------------------------------------------------------------//
	//セグメンテーション方法が3種類選べます
	//1.デプス
	//2.デプス＋テクスチャ
	//3.法線方向
	//ここではim_depth_edgeを上記の3種類のどれかの方法で作成します．
	//--------------------------------------------------------------------------------//
	//セグメンテーションの共通的な変数
	int m_nPrmCannyEdge = 30;
	cv::Mat im_depth_edge = cv::Mat::zeros(im_depth_smooth.rows, im_depth_smooth.cols, CV_8U);
	cv::Mat C =	cv::Mat::ones(3, 3, CV_32FC1);

	if( use_NORMAL_SEG ){
		//法線方向を使ったセグメンテーション
		cv::Mat im_NormalMap;
		int PCASize			= 11;	//	法線推定に使う矩形サイズ(メディアンフィルタのサイズくらいかそれ以上がいいです)
		int ResizeRate		= 4;	//	処理時間に寄与します．画像をリサイズの割合 (Sizeが 1/ResizeRate になる)
		unsigned char range_th	= 140;	// 20160623 飯塚　追加　距離画像のレンジを広げるためのパラメータ（0-255）0が奥で255が手前 (推奨値：140)
		float noise_theta_th	= 65.0; // 20160623 飯塚　追加　床面反射ノイズを消すためのパラメータ（0.0-90.0）小さいほどノイズが消える（推奨値：65.0）

		// top, front, side の3面に分割
		// 注意：閾値がべた書きの状態　視点に応じて変更する必要あり 20160511 
		cv::Mat top = cv::Mat::zeros(im_depth_smooth.rows, im_depth_smooth.cols, CV_8U);	// 20160621 domae 配列確保修正 -特に問題なし
		cv::Mat front = cv::Mat::zeros(im_depth_smooth.rows, im_depth_smooth.cols, CV_8U);
		cv::Mat side = cv::Mat::zeros(im_depth_smooth.rows, im_depth_smooth.cols, CV_8U);

		calcNormalMap_recg( &im_depth_smooth, PCASize, ResizeRate, backgroundID, range_th, noise_theta_th, log_img, &top, &front, &side );	// 20160623 飯塚 range_th, noise_theta_th, log_img の追加，OutImgの削除
		//calcNormalMap_recg( &im_depth_smooth, PCASize, ResizeRate, backgroundID, &im_NormalMap, &top, &front, &side );	// 20160621 入力画像はraw距離画像に修正

		cv::Mat t1, t2, t3;
		cv::Canny(top, t1, m_nPrmCannyEdge/2, m_nPrmCannyEdge, 3 );
		cv::Canny(front, t2, m_nPrmCannyEdge/2, m_nPrmCannyEdge, 3 );
		cv::Canny(side, t3, m_nPrmCannyEdge/2, m_nPrmCannyEdge, 3 );

		cv::bitwise_or( im_depth_edge, t1, im_depth_edge );
		cv::bitwise_or( im_depth_edge, t2, im_depth_edge );
		cv::bitwise_or( im_depth_edge, t3, im_depth_edge );

//		if(1){
//			cv::imshow( "top", t1 );
//			cv::imshow( "front", t2 );
//			cv::imshow( "side", t3 );
//			cv::waitKey(0);
//			cv::imshow( "edge", im_depth_edge );
//			cv::waitKey(0);
//
//		}

		cv::dilate( im_depth_edge, im_depth_edge, C);
	}else{

		//0402b - 0402cまででデプスセグメンテーション
		//
		//[0402b] edge detection
		cv::Canny( im_depth_smooth, im_depth_edge, 0, 15, 3 );
		//cv::imshow( "im_depth_edge", im_depth_edge );
		//cv::waitKey();

		//[0402c] dilation
		cv::dilate( im_depth_edge, im_depth_edge, C);
		//cv::imshow( "im_depth_edge", im_depth_edge );
		//cv::waitKey();


		//////////////////////////////////////////////////////////////
		// 2016.05.19
		// テクスチャセグメンテーション
		// デプスエッジにテクスチャエッジを追加する処理．
		//////////////////////////////////////////////////////////////
		if( use_TEX_SEG ){
			cv::Mat im_grey;
			cv::cvtColor( im_color, im_grey, CV_RGB2GRAY);
			cv::normalize( im_grey, im_grey, 0, 255, cv::NORM_MINMAX );
			cv::medianBlur( im_grey, im_grey, 5 );
			for(int i = 0; i < 2; i++)
			{
				cv::medianBlur( im_grey, im_grey, 15 );
			}
			//cv::imshow( "im_grey", im_grey );
			//cv::waitKey();
			cv::Mat im_grey_edge;
			cv::Canny( im_grey, im_grey_edge, 50, 80, 3 );
			if( log_img ){
				cv::imwrite( "im_gray.bmp", im_grey );
				cv::imwrite( "im_g_edge.bmp", im_grey_edge );
				cv::imwrite( "im_d_edge.bmp", im_depth_edge );
			}
			if( use_viewer ){
				cv::imshow( "im_grey_edge", im_grey_edge );
				cv::waitKey();
			}
			cv::dilate( im_grey_edge, im_grey_edge, C);
			//cv::imshow( "im_grey_edge", im_grey_edge );
			//cv::waitKey();

			if( use_viewer ){
				cv::imshow( "im_depth_edge", im_depth_edge );
				cv::waitKey();
			}
			//ここでデプスエッジにテクスチャエッジを追加する
			for( int j=0; j<im_grey_edge.rows ; j++ ){
				for( int i=0; i<im_grey_edge.cols ; i++ ){
					if( im_grey_edge.at<uchar>( j, i ) == 255 ){
						im_depth_edge.at<uchar>( j, i ) = 255;
					}
				}
			}
			if( use_viewer ){
				cv::imshow( "im_depth_texture_edge", im_depth_edge );
				cv::waitKey();
			}
		}
		//////////////////////////////////////////////////////////////
	}



	cv::imwrite( "im_depth_edge.bmp", im_depth_edge );

	//[0402d] エッジマスク
	cv::Mat im_edge_mask;
	im_edge_mask	=	(im_depth_edge==0) - (im_depth_smooth==0);
	if( use_viewer ){
		cv::imshow( "im_edge_mask", im_edge_mask );
		cv::waitKey();
	}

	//[0402d_] opening 
	cv::Mat im_edge_mask2;
	cv::erode( im_edge_mask, im_edge_mask2, C );
	for( int i=0 ; i<n_closing_cluster-1 ; i++ ){
		cv::erode( im_edge_mask2, im_edge_mask2, C);
	}
	for( int i=0 ; i<n_closing_cluster ; i++ ){
		cv::dilate( im_edge_mask2, im_edge_mask2, C);
	}
	if( use_viewer ){
		cv::imshow( "im_edge_mask2", im_edge_mask2 );
		cv::waitKey();
	}

	//[0402e] ラベリング
	LabelingBS	lab;
	int	n_segment; //セグメント数
	cv::Mat im_label( im_depth.rows, im_depth.cols, CV_16UC1 );
	lab.Exec(
		im_edge_mask.data,
		(short *)&im_label.data[0],
		im_depth.cols,
		im_depth.rows,
		true,
		min_segment_size
		);
	n_segment = lab.GetNumOfResultRegions();
	int lab_num2 = lab.GetNumOfResultRegions();



	// セグメント数の確認
	if( log ) fprintf( fp_logRecg,"n_segment_d: %d\n", n_segment );
	if( log ) fprintf( fp_logRecg,"n_segment_c: %d\n", lab_num2 );


	// セグメンテーション結果の表示
	cv::Mat im_label_nrm;
	cv::normalize( im_label, im_label_nrm, 0, 255, 32, CV_8UC1 );	//正規化
	if( use_viewer ){
		cv::imshow( "im_label", im_label_nrm );
		cv::waitKey();
	}


	//[0403A] 2016.06.20 秋月追加
	//各セグメントの画像を生成する．
	std::vector<cv::Mat> im_seg;
	im_seg.resize( lab_num2+1 );
	for( int k=0 ; k<lab_num2+1 ; k++ ){
		im_seg[k] = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	}
	for( int j=0 ; j<HEIGHT ; j++ ){
		for( int i=0 ; i<WIDTH ; i++ ){
			im_seg[(int)im_label.at<short>( j, i )].at<uchar>( j, i ) = 255;
		}
	}
//	if(1){
//		char seg_name[256];
//		for( int k=0 ; k<lab_num2+1 ; k++ ){
//			sprintf( seg_name,"seg%d.bmp", k );
//			cv::imwrite( seg_name, im_seg[k] );
//		}
//	}

	time2 = static_cast<double>(cv::getTickCount());
	Seg_time = (time2 - time1)*1000 / cv::getTickFrequency();




	//-------------------------------------------------------------------//
	// 把持点を増やす試み
	//-------------------------------------------------------------------//
	// セグメントの0番は背景なので注意すること
	std::vector<cv::Point> seg_center; //セグメント重心座標
	std::vector<int> seg_n_pix;        //セグメント面積
	std::vector<std::vector<cv::Point>> seg_pix; //セグメント座標
	cv::Point tmp_seg_pix;

	if( use_SegCenter ){
		seg_center.resize( lab_num2+1 );
		seg_n_pix.resize( lab_num2+1 );
		seg_pix.resize( lab_num2+1 );
		for( int k=0 ; k<lab_num2+1 ; k++ ){
			seg_center[k].x = 0;
			seg_center[k].y = 0;
			seg_n_pix[k] = 0;
			seg_pix[k].clear();
		}
		for( int j=0 ; j<HEIGHT ; j++ ){
			for( int i=0 ; i<WIDTH ; i++ ){
				seg_center[(int)im_label.at<short>( j, i )].x += i;
				seg_center[(int)im_label.at<short>( j, i )].y += j;
				seg_n_pix[(int)im_label.at<short>( j, i )]++;

				if( (int)im_label.at<short>( j, i ) != 0 ){ //メモリ節約のため，0番（背景）の画素は覚えない．
					tmp_seg_pix.x = i;
					tmp_seg_pix.y = j;
					seg_pix[(int)im_label.at<short>( j, i )].push_back( tmp_seg_pix );
				}
			}
		}
		for( int k=0 ; k<lab_num2+1 ; k++ ){
			seg_center[k].x = (int)( (double)seg_center[k].x / (double)seg_n_pix[k] );
			seg_center[k].y = (int)( (double)seg_center[k].y / (double)seg_n_pix[k] );
		}
	}



	//[0200] Cascaded FAST による把持点認識
	time1 = static_cast<double>(cv::getTickCount());
	double FAST_time;
	if( use_FAST ){
		if( log ) fprintf( fp_logRecg,"Fast_____Start.\n");
		std::vector<Hyp> fast_hyp;
		RunCascadedFast( im_color, im_depth, viewpoint, nItemIndices, &nItemNum, fast_hyp ) ;
		for( int i=0 ; i<fast_hyp.size() ; i++ ){
			hyp.push_back( fast_hyp[i] );
		}
		n_hyp = hyp.size();
		if( log ) fprintf( fp_logRecg,"Fast_____End.\n");
		// FASTの把持点の認識結果の保存
		if( log_img2 ){
			saveHypotheses( &im_color, fast_hyp, "result_FAST.bmp" );
		}

		//長谷川 以下の2行を追加 --2016.06.01
		fast_hyp.clear();
		std::vector<Hyp> ().swap(fast_hyp);
		
	}

	time2 = static_cast<double>(cv::getTickCount());
	FAST_time = (time2 - time1)*1000 / cv::getTickFrequency();
	fprintf( SE,"Fast end\n");

	//[0300] 把持点認識
	time1 = static_cast<double>(cv::getTickCount());
	double FGE_time;
	fprintf( SE,"FGE start\n" );
	std::vector<cv::Point> grasp_v;
	if( !use_SegCenter ){
		//run_FGE() の第3引数は 
		//2:吸着，3:挟持
		if( run_FGE( im_depth, im_color, segMethod, viewpoint, 2, grasp_v, log ) ){
		}
		fprintf( SE,"FGE end\n" );
		for( int i=0 ; i<grasp_v.size() ; i++ ){
			fprintf( SE,"grasp_v( %d, %d )\n", grasp_v[i].x, grasp_v[i].y );
		}
	}
	time2 = static_cast<double>(cv::getTickCount());
	FGE_time = (time2 - time1)*1000 / cv::getTickFrequency();


	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////
	// 把持点を削除して，セグメント重心に入れ替え
	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////
	std::vector<bool> flag_seg_centroid;
	if( use_SegCenter ){
		grasp_v.clear();

		// セグメント重心を識別点にする方法↓↓↓
		//for( int k=1 ; k<lab_num2+1 ; k++ ){
		//	grasp_v.push_back( seg_center[k] );
		//}
		// セグメント重心を識別点にする方法↑↑↑


		//2016.06.23 セグメント上に確実に識別点を生成する方法
		if( log ) fprintf( fp_logRecg,"\n\nGenerate grasp point\n" );
		int grasp_id, n_register;
		for( int k=1 ; k<lab_num2+1 ; k++ ){
			//まずはセグメントの真ん中に一点登録
			//基本はセグメント重心だけど，
			if( (int)im_label.at<short>( seg_center[k].y, seg_center[k].x ) == k ){
				if( log ) fprintf( fp_logRecg,"Seg[%d] -> [centroid]\n", k );
				grasp_v.push_back( seg_center[k] );
				flag_seg_centroid.push_back( true );
			}else{ //重心が穴だったら画素idの真ん中にする．
				if( log ) fprintf( fp_logRecg,"Seg[%d] %d th pix [center id]\n", k, seg_n_pix[k]/2 );
				grasp_v.push_back( seg_pix[k][seg_n_pix[k]/2] );
				flag_seg_centroid.push_back( true );
			}

			n_register = (int)((double)seg_n_pix[k] / (double)grasp_step);
			for( int i=0 ; i<n_register ; i++ ){
				grasp_id = (int)( ((double)rand()/(double)RAND_MAX) * (double)(seg_n_pix[k]-1) );
				grasp_v.push_back( seg_pix[k][grasp_id] );
				flag_seg_centroid.push_back( false );
					
				if( log ) fprintf( fp_logRecg,"Seg[%d] %d th pix total:%d\n", k, grasp_id, seg_n_pix[k] );
			}

			// 予定数の識別点が生成された場合は終了．
			if( max_grasp_point < grasp_v.size() ) break;
		}

		//2016.06.25 把持点が少なすぎる場合は把持点を追加．
		if( grasp_v.size() < min_grasp_point ){
			int loop_cnt; //無限ループに陥らないようにするための変数
			loop_cnt=0;
			while( grasp_v.size() < min_grasp_point ){
				for( int k=1 ; k<lab_num2+1 ; k++ ){
		
					grasp_id = (int)( ((double)rand()/(double)RAND_MAX) * (double)(seg_n_pix[k]-1) );
					grasp_v.push_back( seg_pix[k][grasp_id] );
					flag_seg_centroid.push_back( false );
						
					if( log ) fprintf( fp_logRecg,"Seg[%d] %d th pix total:%d\n", k, grasp_id, seg_n_pix[k] );
				}
				loop_cnt++;
				// ループが10回も回るのはおかしいのでbreakする．
				if( 10 < loop_cnt ){
					if( log ) fprintf( fp_logRecg,"把持点を増やすブロックで無限ループが発生．\n" );
					break;
				}
			}
		}
		
	}


	//把持点が多すぎる場合は削除する．
	//何らかのアルゴリズムがあるといいと思うが，今回は末尾を削除
	if( max_grasp_point < grasp_v.size() ){
		int n_reduce;
		n_reduce = grasp_v.size() - max_grasp_point;
		fprintf( SE,"all grasp point:%d\n", grasp_v.size() );
		fprintf( SE,"n_reduce:%d\n", n_reduce );
		for( int i=0 ; i<n_reduce ; i++ ){
			grasp_v.pop_back();
			flag_seg_centroid.pop_back();
		}
	}
	fprintf( SE,"Reduced grasp point:%d\n", grasp_v.size() );
	for( int i=0 ; i<grasp_v.size() ; i++ ){
		fprintf( SE,"grasp_v( %d, %d )\n", grasp_v[i].x, grasp_v[i].y );
	}



	///* セグメントと識別点の可視化
	if( log_img ){
		char seg_ofname[2048];
		cv::Mat label(im_label.size(), CV_8UC3, cv::Scalar(255, 255, 255));
		for (int i = 0; i < n_segment; i++)
		{
			// ラベリング結果の領域を抽出する。
			cv::Mat labelarea;
			compare(im_label, i+1, labelarea, CV_CMP_EQ);
			//
			// 抽出した領域にランダム色を設定する。
			cv::Scalar randomcolor = cv::Scalar(rand()&0xFF, rand()&0xFF, rand()&0xFF);
			cv::Mat color(im_label.size(), CV_8UC3, randomcolor);
			color.copyTo(label, labelarea);
		}
		for( int i=0 ; i<grasp_v.size() ; i++ ){
			if( flag_seg_centroid[i] ){
				cv::circle( label, grasp_v[i], 8, cv::Scalar( 0, 0, 255 ), -1, CV_AA ); //色の指定はBGRの順
				cv::circle( label, grasp_v[i], 5, cv::Scalar( 255, 255, 255 ), -1, CV_AA ); //色の指定はBGRの順
			}else{
				cv::circle( label, grasp_v[i], 8, cv::Scalar( 255, 0, 0 ), -1, CV_AA ); //色の指定はBGRの順
				cv::circle( label, grasp_v[i], 5, cv::Scalar( 255, 255, 255 ), -1, CV_AA ); //色の指定はBGRの順
			}
		}
		sprintf( seg_ofname, "segment_grasp_point.bmp" );
		imwrite( seg_ofname, label );
	}
	

	//[0300] CNN による把持点認識
	time1 = static_cast<double>(cv::getTickCount());
	double CNN_time;
	if( use_CNN ){
		if( log ) fprintf( fp_logRecg,"CNN_____Start.\n");
		std::vector<Hyp> cnn_hyp;
		fprintf( SE,"CNN_____Start.\n");
		if( RunCNN( im_color, im_depth, nItemIndices, &nItemNum, nTargetItemIdx, grasp_v, cnn_hyp, viewpoint ) ){
			fprintf( SE,"CNN_END.\n");
			for( int i=0 ; i<cnn_hyp.size() ; i++ ){
				hyp.push_back( cnn_hyp[i] );
			}
			n_hyp = hyp.size();
			if( log ) fprintf( fp_logRecg,"CNN_____End.\n");
		}else{
			if( log ) fprintf( fp_logRecg,"!!Error. RunCNN()\n");
		}
		// CNNの把持点の認識結果の保存
		if( log_img2 ){
			saveHypotheses( &im_color, cnn_hyp, "result_CNN.bmp" );
		}

		//長谷川 以下の2行を追加 --2016.06.01
		cnn_hyp.clear();
		std::vector<Hyp> ().swap(cnn_hyp);

	}

	time2 = static_cast<double>(cv::getTickCount());
	CNN_time = (time2 - time1)*1000 / cv::getTickFrequency();

	fprintf( SE,"CNN end\n");


	//長谷川 以下の4行を追加 --2016.06.01
	grasp_v.clear();
	std::vector<cv::Point> ().swap(grasp_v);

	//把持位置のスケーリング（入力画像の縮小比率に合わせとく）
	for( int i=0 ; i<n_hyp ; i++ ){
		hyp[i].grasp.x = (int)(SCALE*(double)hyp[i].grasp.x);
		hyp[i].grasp.y = (int)(SCALE*(double)hyp[i].grasp.y);
	}




	//[01G] 人工的な遺伝子の読み込み
	if( log ) fprintf( fp_logRecg,"人工的な遺伝子の読み込み.\n");
	std::vector<individuall> art_gene;
	if( use_artifitial ){
		if( log ) fprintf( fp_logRecg,"Additional module: use artifitial gene.\n");
		FILE	*fp_art;
		int		n_art, bit;
		if( (fp_art = fopen( artfname, "r" )) == NULL ){
			if( log ){
				fprintf( fp_logRecg,"Artifitial gene is not read.\n");
				fclose( fp_logRecg );
			}
			return false;
		}
		fscanf( fp_art,"%d", &n_art );
		art_gene.resize( n_art );
		for( int j=0 ; j<n_art ; j++ ){
			art_gene[j].chrom.resize( n_hyp );
			art_gene[j].born = MODIFIED;
			for( int i=0 ; i<n_hyp ; i++ ){
				fscanf( fp_art,"%d", &bit );
				if( bit == 1 ) art_gene[j].chrom[i] = true;
				else		   art_gene[j].chrom[i] = false;

			}

		}
	}

	//double time_load_data;
	//time_load_data = timer->getTime() - processing_time;
	//processing_time = timer->getTime() ;

	//[0403] 仮説データに帰属するセグメントidを割り当てる
	time1 = static_cast<double>(cv::getTickCount());
	double Seg_assign_time;
	for( int i=0 ; i<n_hyp ; i++ ){
		hyp[i].segIdx = (int)im_label.at<unsigned short>( hyp[i].grasp.y, hyp[i].grasp.x );
		if( log ) fprintf( fp_logRecg,"hyp:%d -> seg:%d\n", i, hyp[i].segIdx );
		if( log ) fprintf( fp_logRecg,"          grasp( %d, %d )\n", hyp[i].grasp.x, hyp[i].grasp.y );
		if( hyp[i].segIdx == 0 ){
			if( log ) fprintf( fp_logRecg,"need update\n" );
			int new_label;
			cv::Point new_grasp;
			searchNeasestSegment( hyp[i].grasp.x, hyp[i].grasp.y, &im_label, &new_label, &new_grasp );
			hyp[i].segIdx = new_label;
			hyp[i].grasp = new_grasp;
			if( log ) fprintf( fp_logRecg,"    hyp:%d -> seg:%d\n\n", i, hyp[i].segIdx );
			if( log ) fprintf( fp_logRecg,"              grasp( %d, %d )\n", hyp[i].grasp.x, hyp[i].grasp.y );
		}
		//cv::imshow( "segment", hyp[i].segment );
		if( log ) fprintf( fp_logRecg,"grasp(%d,%d)\n", hyp[i].grasp.x, hyp[i].grasp.y );
	}
	//fprintf( SE,"Number of segment: %d\n", n_segment );
	//cv::waitKey();


	//[0404] セグメントが割りあたらなかった仮説を削除
	//       キーポイントと仮説が遠すぎる
	std::vector<Hyp> hyp2;
	for( int i=0 ; i<n_hyp ; i++ ){
		//if( hyp[i].segIdx != 0 ){
		if( (hyp[i].segIdx != 0) && (hyp[i].label != 0) ){ //2016.06.24 秋月 背景ラベルがついた識別点も削除
			hyp2.push_back( hyp[i] );
		}
	}
	hyp.clear();
	for( int i=0 ; i<hyp2.size() ; i++ ){
		hyp.push_back( hyp2[i] );
	}
	n_hyp = hyp.size();
	hyp2.clear();


	time2 = static_cast<double>(cv::getTickCount());
	Seg_assign_time = (time2 - time1)*1000 / cv::getTickFrequency();

	fprintf( SE,"Seg end\n");

	//仮説データがない場合はfalse
	if( hyp.size() == 0 ){

//#ifdef DEBUG_02	// ★★★ 20160621 検証用 
FILE	*fp_deb1=0;
fopen_s(&fp_deb1, "0001_RecgAPC2016_DEBUG_LOG.txt", "w");
fprintf_s(fp_deb1, "仮説データがないのでfalse\n");
fclose(fp_deb1);
//#endif

		fclose( fp_logRecg );
		return false;
	}

	
	//FILE *fp_cnn;
	//if( log ){
	//	fp_cnn = fopen( "cnn.txt", "w" );
	//}
	//2016.06.25 CNNのスコアの補正
	int patch_size;
	patch_size = 165;
	for( int i=0 ; i<n_hyp ; i++ ){
		//if( hyp[i].method == CNN_LABEL ){
		if( (hyp[i].method == CNN_LABEL) 
			&& (hyp[i].label!=36) && (hyp[i].label!=39) ){ //歯ブラシはもともとパッチ内データが少ないので適用外．

			cv::Mat patch;
			int area;
			area = 0;
			//cv::getRectSubPix(im_seg[hyp[i].segIdx], cv::Size(patch_size, patch_size), hyp[i].grasp, patch);
			//セグメントだと細かすぎる場合があるので，距離画像に変更
			cv::getRectSubPix( im_depth_smooth, cv::Size(patch_size, patch_size), hyp[i].grasp, patch );
			area = cv::countNonZero( patch );
			//fprintf( fp_cnn,"hyp[%d] grasp(%d, %d) score: %lf \n", i, hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].score );

			//hyp[i].score = hyp[i].score * tanh( 3.5*(double)((double)area/(double)(patch_size*patch_size)) );
			hyp[i].score = hyp[i].score * pow( (double)((double)area/(double)(patch_size*patch_size)), 1.0/8.0 );
			//fprintf( fp_cnn,"  area: %lf\n", (double)area / (patch_size*patch_size) );
			//fprintf( fp_cnn,"  -> new score: %lf\n", hyp[i].score );
		}
	}
	//fclose( fp_cnn );

	//--------------------------------------------------------------------------------//
	//[05 Genetic Algorithm]
	//--------------------------------------------------------------------------------//
	fprintf( SE,"GA start\n");
	time1 = static_cast<double>(cv::getTickCount());
	double GA_time;

	//[0500] ログ用の変数の用意（GAの処理と関係ないので，処理時間の計算対象外とした．）
	int	log_n_generate; //生成された総個体数

	log_n_generate = population;
	//[0501] 遺伝的アルゴリズムによる認識開始
	if( log ){
		fprintf( fp_logRecg,"Start GA\n" );
		fprintf( fp_logRecg,"Module:[0500]\n" );
	}

	FILE *fp_log;
	if( log ){
		fp_log = fopen( "log.txt", "w" );
	}

	GA	su; //scene understanding
	std::vector<double>	max_fitness;
	std::vector<double>	ave_fitness;
	su.generateInitialIndividuals( population, hyp.size(), n_on ); 

	// [0502] 人工的に作った個体の代入
	if( use_artifitial ){
		for( int i=0 ; i<art_gene.size() ; i++ ){
			su.ind.pop_back();
		}
		for( int i=0 ; i<art_gene.size() ; i++ ){
			su.ind.push_back( art_gene[i] );
		}
		su.showIndividual( population-1 );
		getchar();
	}else{
		if( log ) fprintf( fp_logRecg,"Don't use artifitial gene.\n");
	}


	for( int i=0 ; i<su.getN_ind() ; i++ ){
		calcFitness( &im_label_nrm, hyp, n_item, n_segment, seg_n_pix, &su.ind[i] );
#if LOG_MODE
		su.showIndividual( i );
		//showSceneHypothesis( &im_depth3, hyp, &su.ind[i] );
#endif
	}

	if( use_artifitial ){
		su.showIndividual( su.getN_ind()-1 );
		showSceneHypothesis( &im_depth3, hyp, &su.ind[su.getN_ind()-1], "art_gene.bmp", n_segment, im_seg, im_label );

	}

	su.calcAveFitness();
	su.SortIndividuals();
	max_fitness.push_back( su.ind[0].fitness );
	ave_fitness.push_back( su.getAveFitness() );


	// Start genetic operation.
	for( int i=0 ; i<n_gen ; i++ ){

		if( 20 < rand()%100 ){
			su.crossover();
		}

		if( 97 < rand()%100 ){
			su.mutation();
		}

		// 子個体の適応度計算
//		now = GAtimer->getTime();
		for( int k=0 ; k<su.child.size() ; k++ ){
			calcFitness( &im_label_nrm, hyp, n_item, n_segment, seg_n_pix, &su.child[k] );
		}

//		log_n_generate += su.child.size();
		//su.showIndividual( 0 );

		// 子個体を親の個体集団へ登録
		su.update();
		su.calcAveFitness();

		max_fitness.push_back( su.ind[0].fitness );
		ave_fitness.push_back( su.getAveFitness() );
		//============================
		// 終了条件
		//============================
		// n_stable_gen世代以上変化が無かったら終了する．
		if( n_stable_gen < i ){
			if( max_fitness[i] == max_fitness[i-n_stable_gen] ){
				break;
			}
		}
	}
	if( log ) fclose( fp_log );

	time2 = static_cast<double>(cv::getTickCount());
	GA_time = (time2 - time1)*1000 / cv::getTickFrequency();
	
	//2016.06.25 バックアップ
	individuall ind_cpy;
	ind_cpy.chrom.resize( su.ind[0].chrom.size() );
	for( int i=0 ; i<ind_cpy.chrom.size() ; i++ ){
		ind_cpy.chrom[i] = su.ind[0].chrom[i];
	}
	ind_cpy.fitness = su.ind[0].fitness;

	DeleteMultiLabel( hyp, n_segment, &su.ind[0] );

	fprintf( SE,"GA end\n");
	//--------------------------------------------------------------------------------//
	//[06 返却用の把持点データを出力]
	//--------------------------------------------------------------------------------//
	time1 = static_cast<double>(cv::getTickCount());
	double Out_time;
	//2016.05.19
	//GAの結果から，ターゲットのラベルだけを抜き出す
	std::vector<int> res_i;
	std::vector<int> res_j;
	std::vector<double> res_score;
	std::vector<int> resIdx;
	std::vector<int> hypIdx;
	int	cnt;
	cnt = 0;

	//飯塚　追加　-start
	std::vector<int> nt_res_i;
	std::vector<int> nt_res_j;
	std::vector<double> nt_res_score;
	std::vector<int> nt_resIdx;
	std::vector<int> nt_hypIdx;
	int	nt_cnt;
	nt_cnt = 0;
	//飯塚　追加　-end

	if( log ){
		FILE *fp_hyp;
		fp_hyp = fopen( "all_hyp.txt", "w" );
		fprintf( fp_hyp, " Id, Label\n" );
		for( int i=0 ; i<hyp.size() ; i++ ){
			fprintf( fp_hyp, "%03d, %03d\n", i, hyp[i].label );
		}
		fclose( fp_hyp );
	}

	cv::Mat im_cluster; //クラスタ画像
	cv::Mat im_nt_cluster; //ターゲット以外クラスタ
	cv::Mat im_objIdx;	   //id map
	cv::Mat im_id_mask;
	cv::Mat im_c_map;

	im_cluster = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	im_nt_cluster = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
	im_objIdx = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
	im_id_mask = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
	im_c_map = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

	cv::Mat CC =	cv::Mat::ones(9, 9, CV_32FC1); //膨張用

	if( nTargetItemIdx != 0 ){ //ターゲットが0以外の時（つまり，ビン内の物体認識）


		for( int i=0 ; i<su.ind[0].chrom.size() ; i++ ){
			if( (su.ind[0].chrom[i]) ){
				if( (hyp[i].label == nTargetItemIdx) ){
					if( log ){
						fprintf( fp_logRecg,"%4d,%4d,%d,%d,%.3lf", hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].label, hyp[i].segIdx, hyp[i].score);
						if( hyp[i].method == CNN_LABEL )       fprintf( fp_logRecg,",CNN\n");
						else if( hyp[i].method == FAST_LABEL ) fprintf( fp_logRecg,",Fast\n");
					}
					res_i.push_back( hyp[i].grasp.x );
					res_j.push_back( hyp[i].grasp.y );
					res_score.push_back( hyp[i].score );
					resIdx.push_back( cnt );
					hypIdx.push_back( i );
					cnt++;
				}else if( hyp[i].label != 0 ){ //2016.06.24 秋月 条件追加 nt にも背景を渡さない
					if( log ){
						fprintf( fp_logRecg,"[nt]%4d,%4d,%d,%d,%.3lf", hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].label, hyp[i].segIdx, hyp[i].score);
						if( hyp[i].method == CNN_LABEL )       fprintf( fp_logRecg,",[nt]CNN\n");
						else if( hyp[i].method == FAST_LABEL ) fprintf( fp_logRecg,",[nt]Fast\n");
					}
					nt_itemIdx.push_back( hyp[i].label );
					nt_res_i.push_back( hyp[i].grasp.x );
					nt_res_j.push_back( hyp[i].grasp.y );
					nt_res_score.push_back( hyp[i].score );
					nt_resIdx.push_back( nt_cnt );
					nt_hypIdx.push_back( i );
					nt_cnt++;
				}
			}
		}


		if( CombSort( res_score, resIdx ) ){
			fprintf( SE,        "Sorted result\n");
			if( log ) fprintf( fp_logRecg,"Sorted result\n");
			for( int i=0 ; i<res_score.size() ; i++ ){
				fprintf( SE,        "id: %d, %4d, %4d, %.3lf\n", resIdx[resIdx[i]], res_i[resIdx[i]], res_j[resIdx[i]], res_score[resIdx[i]] );
				if( log ) fprintf( fp_logRecg,"id %d: %d, %4d, %4d, %.3lf\n", resIdx[resIdx[i]], res_i[resIdx[i]], res_j[resIdx[i]], res_score[resIdx[i]] );
			}
		}else{
			if( log ){
				fprintf( fp_logRecg,"ソートのエラー\n");
				fprintf( fp_logRecg,"把持点を信頼度によって並び替えずに出力します．\n");
				for( int i=0 ; i<res_score.size() ; i++ ){
					fprintf( fp_logRecg,"id: %d %d, %4d, %4d, %.3lf\n", resIdx[resIdx[i]], res_i[resIdx[i]], res_j[resIdx[i]], res_score[resIdx[i]] );
				}
			}
		}
		//飯塚　追加 -start

		if( CombSort( nt_res_score, nt_resIdx ) ){
			fprintf( SE,        "[nt]Sorted result\n");
			if( log ) fprintf( fp_logRecg,"[nt]Sorted result\n");
			for( int i=0 ; i<nt_res_score.size() ; i++ ){
				fprintf( SE,        "[nt]%4d, %4d, %.3lf\n", nt_res_i[nt_resIdx[i]], nt_res_j[nt_resIdx[i]], nt_res_score[nt_resIdx[i]] );
				if( log ) fprintf( fp_logRecg,"[nt]%4d, %4d, %.3lf, label: %d\n", nt_res_i[nt_resIdx[i]], nt_res_j[nt_resIdx[i]], nt_res_score[nt_resIdx[i]], hyp[ nt_hypIdx[nt_resIdx[i]] ].label );
			}
		}else{
			if( log ){
				fprintf( fp_logRecg,"[nt]ソートのエラー\n");
				fprintf( fp_logRecg,"[nt]把持点を信頼度によって並び替えずに出力します．\n");
				for( int i=0 ; i<nt_res_score.size() ; i++ ){
					fprintf( fp_logRecg,"[nt]%4d, %4d, %.3lf\n, label: %d", nt_res_i[nt_resIdx[i]], nt_res_j[nt_resIdx[i]], nt_res_score[nt_resIdx[i]], hyp[ nt_hypIdx[nt_resIdx[i]] ].label );
				}
			}
		}
		//飯塚　追加 -end





		//出力ファイルへのデータコピー
		for( int i=0 ; i<res_score.size() ; i++ ){
			work_i.push_back( res_i[resIdx[i]] );
			work_j.push_back( res_j[resIdx[i]] );
			work_score.push_back( res_score[resIdx[i]] );
		}
		//飯塚　追加 -start
		//出力ファイルへのデータコピー
		for( int i=0 ; i<nt_res_score.size() ; i++ ){
			nt_i.push_back( nt_res_i[nt_resIdx[i]] );
			nt_j.push_back( nt_res_j[nt_resIdx[i]] );
			nt_score.push_back( nt_res_score[nt_resIdx[i]] );
		}
		//飯塚　追加 -end



		//セグメント画像（clusterの作成）
		for( int j=0 ; j<res_score.size() ; j++ ){
			cv::bitwise_or( im_cluster, im_seg[ hyp[hypIdx[j]].segIdx ], im_cluster );
		}
		//ターゲット以外のセグメント画像（nt_clusterの作成）
		for( int j=0 ; j<nt_res_score.size() ; j++ ){
			cv::bitwise_or( im_nt_cluster, im_seg[ hyp[nt_hypIdx[j]].segIdx ], im_nt_cluster );
		}


		//物体IDを画素値とした画像を作成する(id_mapの作成)
		for( int j=0 ; j<res_score.size() ; j++ ){
			cv::bitwise_and( im_seg[hyp[hypIdx[j]].segIdx], hyp[hypIdx[j]].label, im_id_mask );
			cv::bitwise_or( im_objIdx, im_id_mask, im_objIdx );
		}
		for( int j=0 ; j<nt_res_score.size() ; j++ ){
			cv::bitwise_and( im_seg[hyp[nt_hypIdx[j]].segIdx], hyp[nt_hypIdx[j]].label, im_id_mask );
			cv::bitwise_or( im_objIdx, im_id_mask, im_objIdx );
		}

		//信頼度マップ（c_mapの作成）
		std::vector<std::vector<int>> segIdxList; //縦軸セグメント数，横軸がセグメントのIdに帰属する仮説のId
		std::vector<double> segMeanScore; //セグメントごとの平均スコア -> c_mapの値になる
		segIdxList.resize( n_segment+1 );
		segMeanScore.resize( n_segment+1 );
		for( int j=0 ; j<segMeanScore.size() ; j++ ){
			segMeanScore[j] = 0.0;
		}
		for( int j=0 ; j<res_score.size() ; j++ ){ //segIdxListの作成
			segIdxList[ hyp[hypIdx[j]].segIdx ].push_back( hypIdx[j] );
			segMeanScore[ hyp[hypIdx[j]].segIdx ] += hyp[hypIdx[j]].score;
		}
		for( int j=0 ; j<nt_res_score.size() ; j++ ){ //segIdxListの作成
			segIdxList[ hyp[nt_hypIdx[j]].segIdx ].push_back( nt_hypIdx[j] );
			segMeanScore[ hyp[nt_hypIdx[j]].segIdx ] += hyp[nt_hypIdx[j]].score;
		}
		for( int j=0 ; j<segMeanScore.size() ; j++ ){
			if( 1 < segIdxList[j].size() ){
				segMeanScore[j] /= (double)segIdxList[j].size();
			}
		}

		for( int j=0 ; j<im_label.rows ; j++ ){
			for( int i=0 ; i<im_label.cols ; i++ ){
				im_c_map.at<uchar>( j, i ) = (unsigned char)( 100.0 * (double)segMeanScore[ im_label.at<short>( j, i ) ] ) ;
			}
		}

		cv::dilate( im_objIdx, im_objIdx, CC, cv::Point(-1,-1), 5 );
		cv::dilate( im_c_map, im_c_map, CC, cv::Point(-1,-1), 5 );

		//ターゲットの領域のみ+100
		for( int j=0 ; j<im_label.rows ; j++ ){
			for( int i=0 ; i<im_label.cols ; i++ ){
				if( im_objIdx.at<uchar>( j, i ) == nTargetItemIdx ){ //ターゲットのIDに対しては＋100する．
					im_c_map.at<uchar>( j, i ) += 100;
				}
			}
		}

	}else{ //ターゲットが0の時（つまり，仮置き台での物体認識）

		std::vector<int> res_label;
		for( int i=0 ; i<su.ind[0].chrom.size() ; i++ ){
			if( (su.ind[0].chrom[i]) ){
				if( (hyp[i].label != 0) ){
					if( log ){
						fprintf( fp_logRecg,"%4d,%4d,%d,%d,%.3lf", hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].label, hyp[i].segIdx, hyp[i].score);
						if( hyp[i].method == CNN_LABEL )       fprintf( fp_logRecg,",CNN\n");
						else if( hyp[i].method == FAST_LABEL ) fprintf( fp_logRecg,",Fast\n");
					}
					res_i.push_back( hyp[i].grasp.x );
					res_j.push_back( hyp[i].grasp.y );
					res_score.push_back( hyp[i].score );
					res_label.push_back( hyp[i].label );
					resIdx.push_back( cnt );
					hypIdx.push_back( i );
					cnt++;
				}
			}
		}

		if( CombSort( res_score, resIdx ) ){
			fprintf( SE,        "Sorted result\n");
			if( log ) fprintf( fp_logRecg,"Sorted result\n");
			for( int i=0 ; i<res_score.size() ; i++ ){
				fprintf( SE,                  "id: %d, %4d, %4d, %.3lf, label = %d\n", resIdx[resIdx[i]], res_i[resIdx[i]], res_j[resIdx[i]], res_score[resIdx[i]], res_label[resIdx[i]] );
				if( log ) fprintf( fp_logRecg,"id: %d, %4d, %4d, %.3lf, label = %d\n", resIdx[resIdx[i]], res_i[resIdx[i]], res_j[resIdx[i]], res_score[resIdx[i]], res_label[resIdx[i]] );
			}
		}

		//セグメント画像（clusterの作成）
		for( int j=0 ; j<res_score.size() ; j++ ){
			cv::bitwise_or( im_cluster, im_seg[hyp[hypIdx[j]].segIdx], im_cluster );
		}
		//id_mapの作成（最大スコアを持つ物体のIDとする）
		im_objIdx = cv::Mat::ones( im_objIdx.rows, im_objIdx.cols, CV_8UC1 ) * res_label[resIdx[0]];
		cv::bitwise_and( im_objIdx, im_cluster, im_objIdx );
		//信頼度マップ（c_mapの作成） 単一の物体だけなので，最大スコアを入れておく
		im_c_map = cv::Mat::ones( im_c_map.rows, im_c_map.cols, CV_8UC1 ) * 200;
		cv::bitwise_and( im_c_map, im_cluster, im_c_map );


		cv::dilate( im_objIdx, im_objIdx, CC, cv::Point(-1,-1), 5 );
		cv::dilate( im_c_map, im_c_map, CC, cv::Point(-1,-1), 5 );

	}



	// 出力データを作成．（ucharの1次元配列にコピー）
	for( int i=0 ; i<im_c_map.cols * im_c_map.rows ; i++ ){
		cluster[i] = im_cluster.data[i];
		nt_cluster[i] = im_nt_cluster.data[i];
		id_map[i] = im_objIdx.data[i];
		c_map[i] = im_c_map.data[i];
	}




	time2 = static_cast<double>(cv::getTickCount());
	Out_time = (time2 - time1)*1000 / cv::getTickFrequency();
	//--------------------------------------------------------------------------------//
	//[07 Post processing]
	//--------------------------------------------------------------------------------//
	time1 = static_cast<double>(cv::getTickCount());
	double Post_time;


	// ログファイルの書き出し
	if( log ){
		if( log_img ){
			showSceneHypothesis( &im_depth, hyp, &su.ind[0], "result.bmp", n_segment, im_seg, im_label );
			//debug_score_MA( &im_label, hyp, &su.ind[0] );
			// テキトーな個体のビットを全部１にして，可視化
			saveAllHypotheses( &im_depth, hyp, &su.ind[0], "result_all.bmp", n_segment, im_seg, im_label );

			showSceneHypothesis( &im_depth, hyp, &ind_cpy, "result_before_delete.bmp", n_segment, im_seg, im_label );
		}
		//最終的な解をテキスト表示
		FILE	*fp_result;
		fp_result = fopen( "result_chrom.txt", "w" );
		for( int i=0 ; i<su.ind[0].chrom.size() ; i++ ){
			if( su.ind[0].chrom[i] ){
				fprintf( fp_result,"1\n");
			}else{
				fprintf( fp_result,"0\n");
			}
		}
		fclose( fp_result );
	

	}

	if( log_img2 ){
		saveSegmentation( im_seg, "result_seg.bmp" );

		cv::imwrite( "output_c_map.bmp", im_c_map );
		cv::imwrite( "output_im_cluster.bmp", im_cluster );
		cv::imwrite( "output_im_nt_cluster.bmp", im_nt_cluster );
		cv::imwrite( "output_id_map.bmp", im_objIdx );

		//ここから output_im_segIdxRGBの作成
		cv::Mat  im_segIdxRGB;
		im_segIdxRGB = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		//飯塚　追加　-start
		cv::Point	cent[40]; //IDとの重心（アイテムIDは40個）
		int			cent_sum[40];
		for( int j = 0; j < 40; j++ ){
			cent[j].x	= 0;
			cent[j].y	= 0;
			cent_sum[j] = 0;
		}
		//飯塚　追加　-end
		for( int j=0 ; j<im_segIdxRGB.rows ; j++ ){
			for( int i=0 ; i<im_segIdxRGB.cols ; i++ ){
				cv::Vec3b	c;
				int	id;
				id = im_objIdx.at<uchar>( j, i );
				//cにid番目の色を割り当てる
				//c(0) = c(1) = c(2) = id; //この行を変更

				//	飯塚　追加　-start
				c(0) = color40[id][2];
				c(1) = color40[id][1];
				c(2) = color40[id][0];

				//重心点算出のための処理
				if( id != 0 ){ //背景は省く
					cent[id].x += i;
					cent[id].y += j;
					cent_sum[id]++;
				}
				//	飯塚　追加　-end

				im_segIdxRGB.at<cv::Vec3b>( j, i ) = c;
			}
		}
		//	飯塚　追加　-start
		//重心点算出
		for( int i = 0; i < 40; i++ ){
			if( cent_sum[i] != 0 ){
				cent[i].x = (int)( (double)cent[i].x / (double)cent_sum[i] );
				cent[i].y = (int)( (double)cent[i].y / (double)cent_sum[i] );
				char id_name[FNL];
				sprintf( id_name, "ID:%d", i );
				cv::putText( im_segIdxRGB, id_name, cent[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3, CV_AA );
				cv::putText( im_segIdxRGB, id_name, cent[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255-color40[i][2], 255-color40[i][1], 255-color40[i][0]), 2, CV_AA );
			}
		}
		//飯塚　追加　-end

		cv::imwrite( "output_im_segIdxRGB.bmp", im_segIdxRGB );



		cv::Vec3b black;
		black(0) = black(1) = black(2) = 0;
		cv::Mat im_su_color = im_color.clone();
		for( int j=0 ; j<im_color.rows ; j++ ){
			for( int i=0 ; i<im_color.cols ; i++ ){
				if( im_cluster.at<uchar>( j, i ) == 0 ){
					im_su_color.at<cv::Vec3b>( j, i ) = black;
				}

			}
		}
		cv::imwrite( "result_cluster_color.bmp", im_su_color );
	}
	if( log2 ){
		//把持点リストの出力(Valid)
		FILE	*fp_result;
		fp_result = fopen( "result_valid_grasp_points.txt", "w" );
		fprintf( fp_result,"i,j,label,segIdx,score,method\n" );
		for( int i=0 ; i<su.ind[0].chrom.size() ; i++ ){
			if( su.ind[0].chrom[i] ){
				fprintf( fp_result,"%d,%d,%d,%d,%.3lf", hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].label, hyp[i].segIdx, hyp[i].score);
				if( hyp[i].method == CNN_LABEL )       fprintf( fp_result,",CNN\n");
				else if( hyp[i].method == FAST_LABEL ) fprintf( fp_result,",Fast\n");
			}
		}
		fclose( fp_result );

		//把持点リストの出力(All)
		fp_result = fopen( "result_all_grasp_points.txt", "w" );
		fprintf( fp_result,"i,j,label,segIdx,score,method\n" );
		for( int i=0 ; i<su.ind[0].chrom.size() ; i++ ){
			fprintf( fp_result,"%d,%d,%d,%d,%.3lf", hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].label, hyp[i].segIdx, hyp[i].score);
			if( hyp[i].method == CNN_LABEL )       fprintf( fp_result,",CNN\n");
			else if( hyp[i].method == FAST_LABEL ) fprintf( fp_result,",Fast\n");
		}
		fclose( fp_result );  // 20160603 川西修正「やってくれるなぁ～byおきひろさん」
	}

	time2 = static_cast<double>(cv::getTickCount());
	Post_time = (time2 - time1)*1000 / cv::getTickFrequency();

	// 処理時間の表示
	time_func_end = static_cast<double>(cv::getTickCount());
	double total_time;
	total_time = (time_func_end - time_func_start)*1000 / cv::getTickFrequency();
	fprintf( SE,"-------------------------------------------\n" );
	fprintf( SE,"Total time : %.1lf [msec]\n", total_time );
	fprintf( SE,"  Seg      : %.1lf [msec]\n", Seg_time );
	fprintf( SE,"  FAST     : %.1lf [msec]\n", FAST_time );
	fprintf( SE,"  FGE      : %.1lf [msec]\n", FGE_time );
	fprintf( SE,"  CNN      : %.1lf [msec]\n", CNN_time );
	fprintf( SE,"  SegAssign: %.1lf [msec]\n", Seg_assign_time );
	fprintf( SE,"  GA       : %.1lf [msec]\n", GA_time );
	fprintf( SE,"  Out      : %.1lf [msec]\n", Out_time );
	fprintf( SE,"  PostProc.: %.1lf [msec]\n", Post_time );
	fprintf( SE,"-------------------------------------------\n" );
	
	
	if( log ){
		fprintf( fp_logRecg,"-------------------------------------------\n" );
		fprintf( fp_logRecg,"Total time: %.1lf [msec]\n", total_time );
		fprintf( fp_logRecg,"  Seg      : %.1lf [msec]\n", Seg_time );
		fprintf( fp_logRecg,"  FAST     : %.1lf [msec]\n", FAST_time );
		fprintf( fp_logRecg,"  FGE      : %.1lf [msec]\n", FGE_time );
		fprintf( fp_logRecg,"  CNN      : %.1lf [msec]\n", CNN_time );
		fprintf( fp_logRecg,"  SegAssign: %.1lf [msec]\n", Seg_assign_time );
		fprintf( fp_logRecg,"  GA       : %.1lf [msec]\n", GA_time );
		fprintf( fp_logRecg,"  Out      : %.1lf [msec]\n", Out_time );
		fprintf( fp_logRecg,"  PostProc.: %.1lf [msec]\n", Post_time );
		fprintf( fp_logRecg,"-------------------------------------------\n" );
	}

	if( log2 ){
		FILE	*fp_time;
		fp_time = fopen( "result_time.txt", "w" );
		fprintf( fp_time,"-------------------------------------------\n" );
		fprintf( fp_time,"Total time: %.1lf [msec]\n", total_time );
		fprintf( fp_time,"  Seg      : %.1lf [msec]\n", Seg_time );
		fprintf( fp_time,"  FAST     : %.1lf [msec]\n", FAST_time );
		fprintf( fp_time,"  FGE      : %.1lf [msec]\n", FGE_time );
		fprintf( fp_time,"  CNN      : %.1lf [msec]\n", CNN_time );
		fprintf( fp_time,"  SegAssign: %.1lf [msec]\n", Seg_assign_time );
		fprintf( fp_time,"  GA       : %.1lf [msec]\n", GA_time );
		fprintf( fp_time,"  Out      : %.1lf [msec]\n", Out_time );
		fprintf( fp_time,"  PostProc.: %.1lf [msec]\n", Post_time );
		fprintf( fp_time,"-------------------------------------------\n" );
		fclose( fp_time );
	}

	if( log ) fclose( fp_logRecg );


	//長谷川 以下の2行を追加 --2016.06.01
	hyp.clear();
	std::vector<Hyp> ().swap(hyp);
	


	return true;
}

//160602 秋月追記 start
//// 入力	cv::Mat	InImg			入力画像(デプス画像)
////		int				PCASize			法線推定に使う矩形サイズ
////		int				ResizeRate		画像をリサイズの割合 (Sizeが 1/ResizeRate になる)
////		int				BackGround		ID視点方向のID
////		unsigned char	range_th		距離画像のレンジを広げるためのパラメータ（0-255）0が奥で255が手前 (推奨値：140)
////		float			noise_theta_th	床面反射ノイズを消すためのパラメータ（0.0-90.0）小さいほどノイズが消える（推奨値：65.0）
////		bool			log_img			logを出す有無



void calcNormalMap_recg( cv::Mat *InImg, int PCASize, int ResizeRate, int BackGroundID, unsigned char range_th, float noise_theta_th, bool log_img, cv::Mat *top, cv::Mat *front, cv::Mat *side ){

	// 入力画像をtmp変数に格納
	cv::Mat InImg_tmp;			//入力のtmp画像
	cv::Mat top_tmp, front_tmp, side_tmp;	//top, front, sideのtmp画像
	InImg_tmp = InImg->clone();
	top_tmp = top->clone();
	front_tmp = front->clone();
	side_tmp = side->clone();

	// サイズ変換画像をresize変数に格納
	cv::Mat InImg_resize;				//入力のresize画像
	cv::Mat top_resize, front_resize, side_resize;	//top, front, sideのresize画像
	cv::resize( InImg_tmp, InImg_resize, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST );
	cv::resize( top_tmp, top_resize, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST );
	cv::resize( front_tmp, front_resize, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST );
	cv::resize( side_tmp, side_resize, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST );

	// デバッグ用変数の初期化
	cv::Mat PlaneImg	= cv::Mat::zeros( InImg_resize.rows, InImg_resize.cols, CV_8UC3 ); //4平面に分割した結果
	cv::Mat ThetaImg	= cv::Mat::zeros( InImg_resize.rows, InImg_resize.cols, CV_8UC1 ); //法線角度を画素値とした画像（グレースケール）
	cv::Mat ThetaColorImg	= cv::Mat::zeros( InImg_resize.rows, InImg_resize.cols, CV_8UC3 ); //法線角度を画素値とした画像（カラー）
	cv::cvtColor( ThetaColorImg, ThetaColorImg, CV_BGR2HSV );

	// z軸方向のレンジを変更(斜め視点のときのみ動作する)
	if( BackGroundID%2 == 0 ){	//視点が斜めであれば
	cv::Mat RangeImg = InImg_resize.clone(); 
		for( int j = 0; j < RangeImg.rows; j++ ){
			for( int i = 0; i < RangeImg.cols; i++ ){
				if( RangeImg.at<uchar>(j, i) < range_th  ){
					RangeImg.at<uchar>(j, i) = range_th-1;
				}
			}
		}
		cv::normalize( RangeImg, InImg_resize, 0, 255, cv::NORM_MINMAX );
		if(log_img) cv::imwrite( "im_range.bmp", RangeImg );
	}

	cv::Vec3b PlaneColor[4];
	PlaneColor[0] = cv::Vec3b(255, 255,   0);
	PlaneColor[1] = cv::Vec3b(  0, 255,   0);
	PlaneColor[2] = cv::Vec3b(  0, 255, 255);
	PlaneColor[3] = cv::Vec3b(  0,   0, 255);

	for( int j = 0; j < InImg_resize.rows; j++ ){
		for( int i = 0; i < InImg_resize.cols; i++ ){
			if( InImg_resize.at<uchar>(j, i) != 0 ){

				// 法線推定　====================================================================================================================
				//pca対象範囲用のMat<pca_patch>を作成
				cv::Mat pca_patch( PCASize*PCASize, 3, CV_16SC1); //-255から255までの値の格納が必要
				int patch_count = 0;
				for( int jj = -(PCASize/2); jj <= (PCASize/2); jj++ ){
					for( int ii = -(PCASize/2); ii <= (PCASize/2); ii++ ){
						if( (0 < jj+j) && (0 < ii+i) && (jj+j < InImg_resize.rows) && (ii+i < InImg_resize.cols) ){	//画像外から出ることを抑制
							if( InImg_resize.at<uchar>(j+jj, i+ii) != 0 ){		//計測値無しはとばす	
								pca_patch.at<short>(patch_count, 0) = ii;
								pca_patch.at<short>(patch_count, 1) = jj;
								pca_patch.at<short>(patch_count, 2) = InImg_resize.at<uchar>(j+jj, i+ii) - InImg_resize.at<uchar>(j, i);
								patch_count++;
							}else{
								pca_patch.at<short>(patch_count, 0) = NULL;
								pca_patch.at<short>(patch_count, 1) = NULL;
								pca_patch.at<short>(patch_count, 2) = NULL;
								patch_count++;
							}
						}else{
								pca_patch.at<short>(patch_count, 0) = NULL;
								pca_patch.at<short>(patch_count, 1) = NULL;
								pca_patch.at<short>(patch_count, 2) = NULL;
								patch_count++;

						}
					}
				}
				// pca_patch.resize( patch_count, patch_count ); 20160620 OpenCV2.1 では pca_patch.resizeができないためほかの処理で代用

				if( patch_count > 10 ){ //PCAの対象範囲内の点群が10個以下であればとばす
					cv::PCA pca( pca_patch, cv::Mat(), CV_PCA_DATA_AS_ROW, 3 ); //PCAの実行

					cv::Vec3f N_Vec; //法線
					N_Vec.val[0] = pca.eigenvectors.at<float>(2, 0);	//法線のx成分
					N_Vec.val[1] = pca.eigenvectors.at<float>(2, 1);	//法線のy成分
					N_Vec.val[2] = pca.eigenvectors.at<float>(2, 2);	//法線のz成分

					if( N_Vec.val[2] < 0 ){ //z軸方向が負（視点と逆を向いていたら）
						N_Vec.val[0] = -N_Vec.val[0];
						N_Vec.val[1] = -N_Vec.val[1];
						N_Vec.val[2] = -N_Vec.val[2];
					}

				// ==============================================================================================================================

				// 出力画像作成　================================================================================================================
					//20160622 飯塚追加（見え方向を考慮） -start
					cv::Vec3f VN_Vec;		// カメラ視点から注目点を結ぶ単位ベクトル
					float VN_VecSca;		// VN_Vec のノルム
					float VN_dot;			// N_vec と VN_Vec との内積
					float dot_theta;		// N_Vec と VN_Vec がなす角度

					VN_Vec.val[0] = (InImg_resize.cols / 2) - i;
					VN_Vec.val[1] = (InImg_resize.rows / 2) - j;
					VN_Vec.val[2] = 500 - (InImg_resize.at<uchar>(j, i));
					VN_VecSca = sqrt( (double)VN_Vec.val[0]*VN_Vec.val[0] + (double)VN_Vec.val[1]*VN_Vec.val[1] + (double)VN_Vec.val[2]*VN_Vec.val[2] );
					VN_Vec.val[0] = VN_Vec.val[0] / VN_VecSca; 
					VN_Vec.val[1] = VN_Vec.val[1] / VN_VecSca; 
					VN_Vec.val[2] = VN_Vec.val[2] / VN_VecSca; 
					VN_dot = VN_Vec.dot(N_Vec);
					dot_theta = (acosf(VN_dot) / CV_PI) * 180.0;

					if( dot_theta < noise_theta_th ){

						//20160622 飯塚追加（見え方向を考慮） -end

						/*//20160622 飯塚追加（見え方向を考慮）コメントアウト -start
						// 20160621 飯塚追加（床反射ノイズを抑制）　-start
						float thetaz_th	= 60.0;		// z 軸方向の角度制限パラメータ（小さいほどノイズが消えやすい） 
						float thetay_th	= 30.0;		// y 軸方向の角度制限パラメータ（大きいほどノイズが消えやすい）
						float thetaz, thetay;	
						thetaz = (acosf(N_Vec.val[2]) / CV_PI ) * 180.0;
						thetay = (acosf(fabs(N_Vec.val[1])) / CV_PI) * 180.0;

						//if( (thetaz_th < thetaz ) && (thetay < thetay_th) ){
						//}else{												
						*///20160622 飯塚追加（見え方向を考慮）コメントアウト -end
						// 20160621 飯塚追加（床反射ノイズを抑制）　-end

						// 法線を画像平面上に投影した角度（rad）を算出
						float rad;
						float N_Sxy;
						N_Sxy = sqrt( N_Vec.val[0]*N_Vec.val[0] + N_Vec.val[1]*N_Vec.val[1] );
						rad = acosf( (float)N_Vec.val[0]/(float)(N_Sxy+0.000001) );
						if( N_Vec.val[1] < 0 ){ //y軸が第3,4象限にあったら
							rad = (M_PI*2.0) - rad;
						}

						if( log_img ){
							// HSV色空間を利用して<ThetaColorImg>にマッピング
							// H:法線角度　S:z軸成分　V:固定値（255）
							ThetaColorImg.at<cv::Vec3b>(j, i).val[0] = (uchar)(((double)rad / ((double)M_PI*2.0)) * 180.0 + 0.5) ;
							ThetaColorImg.at<cv::Vec3b>(j, i).val[1] = (uchar)((N_Vec.val[2] * 255.0) + 0.5);
							ThetaColorImg.at<cv::Vec3b>(j, i).val[2] = 255;
						}

						if( log_img ){
							// 法線角度だけを使ったグレースケール画像作成<ThetaImg>
							ThetaImg.at<uchar>(j, i) = (uchar)(((double)rad / ((double)M_PI*2.0)) * 255.0 + 0.5) ;
						}

						// 4つの平面に割り当てる<PlaneImg>
						float	PlaneVec[4][3];		//4つの平面の法線ベクトル
						float	InPro;				//PlaneVecとの内積値
						float	InProMax = 0;		//InProの最大値
						int		PlaneID  = 0;		//平面ID(0:前面　1:上面　2:左面　4:右面)

						if( BackGroundID%2 == 0 ){	//視点が斜めであれば
							PlaneVec[0][0] =  0.0; PlaneVec[0][1] =  0.70710; PlaneVec[0][2] = 0.70710;	//前面
							PlaneVec[1][0] =  0.0; PlaneVec[1][1] = -0.70710; PlaneVec[1][2] = 0.70710;	//上面
							PlaneVec[2][0] = -1.0; PlaneVec[2][1] =  0.0;	  PlaneVec[2][2] =  0.0;	//左面
							PlaneVec[3][0] =  1.0; PlaneVec[3][1] =  0.0;	  PlaneVec[3][2] =  0.0;	//右面
						}else{						//視点が正面であれば
							PlaneVec[0][0] =  0.0; PlaneVec[0][1] =  0.0; PlaneVec[0][2] = 1.0;	//前面
							PlaneVec[1][0] =  0.0; PlaneVec[1][1] = -1.0; PlaneVec[1][2] = 0.0;	//上面
							PlaneVec[2][0] = -1.0; PlaneVec[2][1] =  0.0; PlaneVec[2][2] = 0.0;	//左面
							PlaneVec[3][0] =  1.0; PlaneVec[3][1] =  0.0; PlaneVec[3][2] = 0.0;	//右面
						}

						for( int k = 0; k < 4; k++ ){
							InPro = 0.0;
							for( int l = 0; l < 3; l++ ){
								InPro += (float)PlaneVec[k][l]*(float)N_Vec.val[l];
							}
							if( InProMax < InPro ){
								InProMax = InPro;
								PlaneID  = k;
							}
						}
						if(log_img){
							PlaneImg.at<cv::Vec3b>(j, i) = PlaneColor[PlaneID];
						}
					
						// <top> <front> <side> の作成
						if( PlaneID == 0 )	front_resize.at<uchar>(j, i)	= 255;
						else if( PlaneID == 1 ) top_resize.at<uchar>(j, i)	= 255;
						else if( PlaneID == 2 ) side_resize.at<uchar>(j, i)	= 255;
						else if( PlaneID == 3 ) side_resize.at<uchar>(j, i)	= 255;

					}// 20160621 飯塚追加（床反射ノイズを抑制）

				}
				// 出力画像作成　================================================================================================================

			}
		}
	}

	//top, front, side 画像の resize
	cv::resize( top_resize, *top, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST );
	cv::resize( front_resize, *front, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST );
	cv::resize( side_resize, *side, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST );

	if( log_img ){
		//デバッグ用変数の resize と imwrite
		cv::Mat ThetaColorImg_ans;
		cv::Mat PlaneImg_ans;
		cv::Mat ThetaImg_ans;
		cv::cvtColor( ThetaColorImg, ThetaColorImg, CV_HSV2BGR );
		cv::resize( ThetaColorImg, ThetaColorImg_ans, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST );
		cv::resize( ThetaImg, ThetaImg_ans, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST );
		cv::resize( PlaneImg, PlaneImg_ans, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST );
		cv::imwrite( "ThetaColorImg.bmp", ThetaColorImg_ans );
		cv::imwrite( "Plane.bmp", PlaneImg_ans );
		cv::imwrite( "Theta.bmp", ThetaImg_ans );
		//出力画像を指定（現在は<ThetaImg>）
		//*OutImg = ThetaImg_ans; 
	}

}
// 飯塚　追加　-end

//160602 秋月追記 end
