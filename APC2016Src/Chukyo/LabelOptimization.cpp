#include "LabelOptimization.h"

#include "ColorTableRGB.h"


// 2線分の交差判定 trueなら交差，falseで交差なし
bool CheckIntersectionOfPairedLineSeg( cv::Point l1s, cv::Point l1e, cv::Point l2s, cv::Point l2e ){

	int ta, tb;
	bool judge1;
	ta = ( l2e.x - l2s.x )*( l1s.y - l2e.y ) + ( l2e.y - l2s.y )*( l2e.x - l1s.x );
	tb = ( l2e.x - l2s.x )*( l1e.y - l2e.y ) + ( l2e.y - l2s.y )*( l2e.x - l1e.x );
	if( ((ta<0)&&(0<tb)) || ((tb<0)&&(0<ta)) ) judge1 = true;
	else judge1 = false;


	int tc, td;
	bool judge2;
	tc = ( l1e.x - l1s.x )*( l2s.y - l1e.y ) + ( l1e.y - l1s.y )*( l1e.x - l2s.x );
	td = ( l1e.x - l1s.x )*( l2e.y - l1e.y ) + ( l1e.y - l1s.y )*( l1e.x - l2e.x );
	if( ((tc<0)&&(0<td)) || ((td<0)&&(0<tc)) ) judge2 = true;
	else judge2 = false;

	if( judge1 && judge2 ){
		return true;
	}else{
		return false;
	}

}


void calc_score_MA( std::vector<Hyp>& hyp, int n_segment, individuall *ind, double *score ){

	int n_max_item;
	n_max_item = 50;

	//縦：セグメント数，横：アイテム種類数のテーブルを作成
	//Trueになれば，そのセグメントにアイテムがあることがわかる．
	std::vector<std::vector<int>> lab_on_seg;
	lab_on_seg.resize( n_segment+1 );
	for( int i=0 ; i<lab_on_seg.size() ; i++ ){
		lab_on_seg[i].resize( n_max_item );
	}
	for( int j=0 ; j<lab_on_seg.size() ; j++ ){  //テーブルの初期化
		for( int i=0 ; i<lab_on_seg[j].size() ; i++ ){
			lab_on_seg[j][i] = 0;
		}
	}

	for( int i=0 ; i<ind->chrom.size() ; i++ ){
		if( ind->chrom[i] ){
			lab_on_seg[ hyp[i].segIdx ][ hyp[i].label ]++;
		}
	}

	int n_valid_segment;
	double	sum_score;
	n_valid_segment = 0;
	sum_score = 0.0;
	for( int j=0 ; j<lab_on_seg.size() ; j++ ){
		int kind_item, n_label;
		kind_item = n_label = 0;
		for( int i=0 ; i<lab_on_seg[j].size() ; i++ ){
			if( 0 < lab_on_seg[j][i] ){
				kind_item++;
				n_label += lab_on_seg[j][i];
			}
		}

		//fprintf( SE,"seg[%d] has %d label.\n", j, kind_item );
		double tmp_score;
		if( 1 < kind_item ){
			tmp_score = 1.0/(double)n_label;
			sum_score += tmp_score;
			n_valid_segment++;
		}else if( 1 == kind_item ){
			tmp_score = 1.0;
			sum_score += tmp_score;
			n_valid_segment++;
		}
	}

	if( 0 < n_valid_segment ){
		*score = sum_score / (double)n_valid_segment;
	}else{
		*score = 0.0;
	}
	//fprintf( SE," Score: %lf\n", *score );


}

void calc_score_NL( std::vector<Hyp>& hyp, individuall *ind, double *score ){

	std::vector<int> label;
	label.resize( 40 ); //物体数が最大（39+背景）個まで対応．
	for( int i=0 ; i<label.size() ; i++ ){
		label[i] = 0;
	}

	int cnt;
	cnt = 0;
	for( int i=0 ; i<ind->chrom.size() ; i++ ){
		if( ind->chrom[i] ){
			//label[ hyp[i].label ]++;
			cnt++;
		}
	}

	*score = (double)((double)cnt / (double)ind->chrom.size());

	//double tmp_score;
	//int	n_valid;
	//tmp_score = 0.0;
	//n_valid = 0;
	//for( int i=0 ; i<label.size() ; i++ ){
	//	if( label[i] != 0 ){
	//		//tmp_score += 1.0 - ( 1.0 / exp((double)label[i]) );
	//		tmp_score += 1.0 - ( 1.0 / (double)label[i] );
	//		//tmp_score += (double)label[i];
	//		n_valid++;
	//	}
	//}

	//*score = (double)( tmp_score / (double)n_valid );
	//*score = (double) tmp_score;

}

void calc_score_LR( std::vector<Hyp>& hyp, int n_item, individuall *ind, std::vector<std::vector<double>>& label_distance, std::vector<double>& obj_size, double *score ){

	int n_hyp;
	int n_max_item;
	n_max_item = 50;
	n_hyp = hyp.size();
	// ラベルごとに有効な仮説番号を取り出す．
	std::vector<std::vector<int>> label_table;
	label_table.resize( n_max_item ); //物体数が最大50個まで対応．
	for( int i=0 ; i<n_hyp ; i++ ){
		if( ind->chrom[i] ){ // 有効な仮説を探す
			label_table[ hyp[i].label ].push_back( i );
		}
	}


	double dist_score;
	int n_valid_label;
	dist_score = 0.0;
	n_valid_label = 0;
	for( int k=0 ; k<n_max_item ; k++ ){
		if(       label_table[k].size() <= 1 ){
			//スコアなし
			//fprintf( SE,"Size of label_table[%d] = %d\n", k, label_table[k].size() );
		}else{
			double label_dist; //ペアの最大距離を算出
			label_dist = 0;
			//fprintf( SE,"Size of label_table[%d] = %d\n", k, label_table[k].size() );
			for( int j=0 ; j<label_table[k].size() ; j++ ){
				for( int i=j+1 ; i<label_table[k].size() ; i++ ){
					//fprintf( SE,"Label pair: %d, %d\n", label_table[k][j], label_table[k][i] );
					//fprintf( SE," Dist: %lf\n", label_distance[ label_table[k][j] ][ label_table[k][i] ] );
					if( label_dist < label_distance[ label_table[k][j] ][ label_table[k][i] ] ){
						label_dist = label_distance[ label_table[k][j] ][ label_table[k][i] ];
					}
				}
			}

			double	dist_diff;
			dist_diff = label_dist - obj_size[k];
			//fprintf( SE,"  Label Dist: %lf\n", label_dist );
			if( dist_diff <= 1.0 ){
				dist_score += 1.0;
			}else{
				dist_score += 1.0/dist_diff;
			}
			n_valid_label++;
		}
	}

	if( 0 < n_valid_label ){
		*score = dist_score / (double)n_valid_label;
	}else{
		*score = 0.0;
	}
	//fprintf( SE,"\n  Score: %lf\n", *score );

}

void calc_score_PC( cv::Mat *im_scene, std::vector<Hyp>& hyp, individuall *ind, double *score ){

	int	n_all_item;
	n_all_item = 40;
	std::vector<std::vector<int>> label_list;
	label_list.resize( n_all_item ); //物体数が最大（38+背景）個まで対応．
	for( int i=0 ; i<label_list.size() ; i++ ){
		label_list[i].clear();
	}
	// ラベルごとに仮説のIDを保存
	for( int i=0 ; i<ind->chrom.size() ; i++ ){
		if( ind->chrom[i] ){
			label_list[ hyp[i].label ].push_back( i );
		}
	}

	std::vector<std::vector<int>> line_seg_list_sp;
	std::vector<std::vector<int>> line_seg_list_ep;
	line_seg_list_sp.resize( n_all_item );
	line_seg_list_ep.resize( n_all_item );
	std::vector<int> one_seg;
	for( int i=0 ; i<n_all_item ; i++ ){
		if( 2 <= label_list[i].size() ){
			for( int jj=0 ; jj<label_list[i].size() ; jj++ ){
				for( int ii=jj+1 ; ii<label_list[i].size() ; ii++ ){
					line_seg_list_sp[i].push_back( label_list[i][jj] );
					line_seg_list_ep[i].push_back( label_list[i][ii] );
				}
			}
		}
		if( 1 == label_list[i].size() ){
			one_seg.push_back( label_list[i][0] );
		}
	}

	int kind_line_seg;
	kind_line_seg = line_seg_list_sp.size();
	bool flag;
	int	 n_check, n_intersection;
	n_check = n_intersection = 0;
	for( int i=0 ; i<n_all_item ; i++ ){
		for( int ii=0 ; ii<line_seg_list_sp[i].size() ; ii++ ){

			for( int j=i+1 ; j<n_all_item ; j++ ){
				for( int jj=0 ; jj<line_seg_list_sp[j].size() ; jj++ ){

					flag =  CheckIntersectionOfPairedLineSeg( hyp[ line_seg_list_sp[i][ii] ].grasp,
															  hyp[ line_seg_list_ep[i][ii] ].grasp,
															  hyp[ line_seg_list_sp[j][jj] ].grasp,
															  hyp[ line_seg_list_ep[j][jj] ].grasp );
					if( flag ) n_intersection++;
					n_check++;
				}
			}
		}
	}

	int	margin;
	margin = 3; //これ以上のラベルの接近はペナルティ
	for( int i=0 ; i<one_seg.size() ; i++ ){
		for( int ii=0 ; ii<label_list.size() ; ii++ ){
			for( int jj=0 ; jj<label_list[ii].size() ; jj++ ){
				if( one_seg[i] != label_list[ii][jj] ){
					cv::Point p1, p2;
					int dist_x, dist_y;
					p1 = hyp[ one_seg[i] ].grasp;
					p2 = hyp[ label_list[ii][jj] ].grasp;

					dist_x = abs( p1.x - p2.x );
					dist_y = abs( p1.y - p2.y );
					if( (dist_x<margin) && (dist_y<margin) ){
						n_intersection++;
					}
					n_check++;
				}
			}
		}
	}

	if( n_check == 0 ) *score = 0.5;
	else			   *score = 1.0 - (double)( (double)n_intersection / (double)n_check );

		//ここから↓は完全にデバッグ処理なので，本番はコメントアウトすること
	//int	width, height;
	//width = im_scene->cols;
	//height = im_scene->rows;

	////カラー画像としてのシーンを作成
	//cv::Mat im_debug = cv::Mat::zeros( height, width, CV_8UC3 );
	//for( int j=0 ; j<height ; j++ ){
	//	for( int i=0 ; i<width ; i++ ){
	//		cv::Vec3b tmp;
	//		tmp(0) = tmp(1) = tmp(2) = im_scene->at<uchar>( j, i );
	//		im_debug.at<cv::Vec3b>( j, i ) = tmp;
	//	}
	//}
	//cv::resize( im_debug, im_debug, cv::Size(), 2.0, 2.0 );
	////線の描画
	//cv::Point text_location;
	//char text[2048];
	//for( int i=0 ; i<n_all_item ; i++ ){
	//	for( int ii=0 ; ii<line_seg_list_sp[i].size() ; ii++ ){

	//		for( int j=i+1 ; j<n_all_item ; j++ ){
	//			for( int jj=0 ; jj<line_seg_list_sp[j].size() ; jj++ ){

	//				cv::line( im_debug, hyp[ line_seg_list_sp[i][ii] ].grasp, hyp[ line_seg_list_ep[i][ii] ].grasp, cv::Scalar( 0, 255, 0 ) );
	//				cv::line( im_debug, hyp[ line_seg_list_sp[j][jj] ].grasp, hyp[ line_seg_list_ep[j][jj] ].grasp, cv::Scalar( 0, 255, 0 ) );
	//				cv::circle( im_debug, hyp[ line_seg_list_sp[i][ii] ].grasp, 3, cv::Scalar( 255, 255, 0 ) );
	//				cv::circle( im_debug, hyp[ line_seg_list_ep[i][ii] ].grasp, 3, cv::Scalar( 255, 255, 0 ) );
	//				cv::circle( im_debug, hyp[ line_seg_list_sp[j][jj] ].grasp, 3, cv::Scalar( 0, 255, 255 ) );
	//				cv::circle( im_debug, hyp[ line_seg_list_ep[j][jj] ].grasp, 3, cv::Scalar( 0, 255, 255 ) );


	//				text_location.x = hyp[ line_seg_list_sp[i][ii] ].grasp.x;
	//				text_location.y = hyp[ line_seg_list_sp[i][ii] ].grasp.y-10;
	//				sprintf( text,"%d", hyp[ line_seg_list_sp[i][ii] ].label );
	//				cv::putText( im_debug, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 2, CV_AA);
	//				cv::putText( im_debug, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA);

	//				text_location.x = hyp[ line_seg_list_ep[i][ii] ].grasp.x;
	//				text_location.y = hyp[ line_seg_list_ep[i][ii] ].grasp.y-10;
	//				sprintf( text,"%d", hyp[ line_seg_list_ep[i][ii] ].label );
	//				cv::putText( im_debug, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 2, CV_AA);
	//				cv::putText( im_debug, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA);

	//				text_location.x = hyp[ line_seg_list_sp[j][jj] ].grasp.x;
	//				text_location.y = hyp[ line_seg_list_sp[j][jj] ].grasp.y-10;
	//				sprintf( text,"%d", hyp[ line_seg_list_sp[j][jj] ].label );
	//				cv::putText( im_debug, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 2, CV_AA);
	//				cv::putText( im_debug, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA);

	//				text_location.x = hyp[ line_seg_list_ep[j][jj] ].grasp.x;
	//				text_location.y = hyp[ line_seg_list_ep[j][jj] ].grasp.y-10;
	//				sprintf( text,"%d", hyp[ line_seg_list_ep[j][jj] ].label );
	//				cv::putText( im_debug, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 2, CV_AA);
	//				cv::putText( im_debug, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA);
	//			}
	//		}
	//	}
	//}
	//for( int i=0 ; i<one_seg.size() ; i++ ){
	//	cv::circle( im_debug, hyp[ one_seg[i] ].grasp, 3, cv::Scalar( 0, 0, 255 ) );
	//	text_location.x = hyp[ one_seg[i] ].grasp.x+10;
	//	text_location.y = hyp[ one_seg[i] ].grasp.y-10;
	//	sprintf( text,"%d", hyp[ one_seg[i] ].label );
	//	cv::putText( im_debug, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 255 ), 2, CV_AA);
	//	cv::putText( im_debug, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA);
	//}
	//text_location.x = 10;
	//text_location.y = 20;
	//sprintf( text,"n_intersection:%d", n_intersection );
	//cv::putText( im_debug, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 2, CV_AA);
	//cv::putText( im_debug, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA);
	////cv::imshow( "Line pair", im_debug );
	//cv::imwrite( "line_pair.bmp", im_debug );
	////cv::waitKey();

}

void calc_score_LC( std::vector<Hyp>& hyp, int n_segment, individuall *ind, double *score ){

	int n_hyp;
	std::vector<double> sum_score;
	std::vector<int> used_seg;  //使用したセグメントid
	n_hyp = hyp.size();

	sum_score.resize( n_segment+1 );
	used_seg.resize( n_segment+1 );
	for( int i=0 ; i<sum_score.size() ; i++ ){
		sum_score[i] = 0.0;
		used_seg[i] = 0;
	}

	for( int i=0 ; i<n_hyp ; i++ ){
		if( ind->chrom[i] ){ // 有効な仮説を探す
			sum_score[ hyp[i].segIdx ] += (double)hyp[i].score;
			used_seg[ hyp[i].segIdx ]++;
		}
	}

	double total_score;
	int cnt;
	total_score = 0.0;
	cnt = 0;
	for( int i=0 ; i<sum_score.size() ; i++ ){
		if( 0<used_seg[i] ){
			total_score += (double)(sum_score[i]/(double)used_seg[i]);
			cnt++;
		}
	}


	if( 0 < cnt ){
		*score = total_score / (double)cnt;
	}else{
		*score = 0.0;
	}
	//fprintf( SE,"\n  Score: %lf\n", *score );

}

void calc_score_UA( std::vector<Hyp>& hyp, std::vector<int>& seg_n_pix, individuall *ind, double *score ){

	int n_hyp;
	n_hyp = hyp.size();
	std::vector<bool> valid_seg_id; //解釈済みセグメントID
	std::vector<bool> used_seg_id; //仮説が乗っているセグメントID


	//解釈済みセグメントIDを探す
	valid_seg_id.resize( seg_n_pix.size() );
	used_seg_id.resize( seg_n_pix.size() );
	for( int i=0 ; i<valid_seg_id.size() ; i++ ){
		valid_seg_id[i] = false;
		used_seg_id[i] = false;
	}
	for( int i=0 ; i<n_hyp ; i++ ){
		if( ind->chrom[i] ){ // 有効な仮説を探す
			valid_seg_id[ hyp[i].segIdx ] = true;
		}
		used_seg_id[ hyp[i].segIdx ] = true;
	}

	//セグメント総面積と解釈済みセグメント面積を計算
	//un_used_areaは仮説がないセグメントのエリア
	int	sum_seg_area, understood_area, un_used_area;
	sum_seg_area = understood_area = un_used_area = 0;
	// 1はじまりなのは，背景を無視するため．
	//fprintf( SE,"seg:");
	for( int i=1 ; i<seg_n_pix.size() ; i++ ){
		sum_seg_area += seg_n_pix[i];
		if( valid_seg_id[i] ){
			understood_area += seg_n_pix[i];
			//fprintf( SE,"1");
		}else{
			//fprintf( SE,"0");
		}

		if( !used_seg_id[i] ){
			un_used_area += seg_n_pix[i];
		}
	}

	*score = (double)( (double)understood_area / (double)(sum_seg_area-un_used_area) );
	//fprintf( SE," Score: %d/%d = %lf\n", understood_area, sum_seg_area-un_used_area, *score );

}

void calcFitness( cv::Mat *im_scene, std::vector<Hyp>& hyp, int n_item, int n_segment, std::vector<int>& seg_n_pix, individuall *ind ){


	double score_MA; //Multiple assign -> セグメント上のラベルの矛盾を評価
	double score_NL; //Number of Labels -> 使ったラベルの種類数を評価
	double score_UA; //Understanded Area -> 有効ラベルの総面積
	double score_PC;
	double score_LC;

	
	score_MA = score_NL = score_UA = score_PC = 0.0;
	calc_score_NL( hyp, ind, &score_NL );
	calc_score_LC( hyp, n_segment, ind, &score_LC );
//	double processing_time;
//	processing_time = 0.0;
	//pcl::ScopeTime::StopWatch *timer( new pcl::ScopeTime::StopWatch );
	//timer->reset();

	calc_score_MA( hyp, n_segment, ind, &score_MA );
	calc_score_UA( hyp, seg_n_pix, ind, &score_UA );

//	time_UA += timer->getTime() - processing_time;
//	processing_time = timer->getTime() ;

	calc_score_PC( im_scene, hyp, ind, &score_PC );

//	time_P += timer->getTime() - processing_time;
//	processing_time = timer->getTime() ;

	ind->score_MA = score_MA;
	ind->score_NL = score_NL;
	ind->score_UA = score_UA;
	ind->score_PC = score_PC;
	ind->score_LC = score_LC;

	//ind->fitness = score_MA + score_NL + score_LR + (0.5*score_UA);
	//ind->fitness = score_UA + (0.3*score_P);
	//ind->fitness = score_MA + (0.5*score_NL) + (0.5*score_PC) + 0.3*score_LC;
	//ind->fitness = score_MA + (0.5*score_NL) + (0.3*score_UA) + (0.3*score_PC) + (0.3*score_LC);
	ind->fitness = score_MA + score_NL + score_UA + (0.5*score_PC) + score_LC;
	//ind->fitness = score_MA + (0.5*score_UA) + (0.3*score_PC) + (0.5*score_LC);
	//ind->fitness =  score_NL;

	//CNT++;
}

void showSceneHypothesis( cv::Mat *im_scene, std::vector<Hyp>& hyp, individuall *ind, char *name, int n_segment, std::vector<cv::Mat>& seg_img, cv::Mat im_label ){

	cv::Mat im_sh = cv::Mat::zeros( im_scene->rows, im_scene->cols, CV_8UC3 );
	char text[FNL];

	// セグメントの描画（文字とか書くときは，この後にする）

	//im_sh = im_scene->clone();
	for( int j=0 ; j<im_sh.rows ; j++ ){
		for( int i=0 ; i<im_sh.cols ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = tmp(1) = tmp(2) = im_scene->at<uchar>( j, i );
			im_sh.at<cv::Vec3b>( j, i ) = tmp;
		}
	}

	for( int i=0 ; i<ind->chrom.size() ; i++ ){
		if( ind->chrom[i] ){
			int t_id;
			t_id = rand()%16;
			for( int jj=0 ; jj<im_sh.rows ; jj++ ){
				for( int ii=0 ; ii<im_sh.cols ; ii++ ){
					if( seg_img[hyp[i].segIdx].at<uchar>( jj, ii ) == 255 ){
						cv::Vec3b tmp;
						tmp(2) = color16[t_id][0]; tmp(1) = color16[t_id][1]; tmp(0) = color16[t_id][2]; 
						im_sh.at<cv::Vec3b>( jj, ii ) = tmp;
					}
				}
			}
			
		}
	}

	// 飯塚　追加　-start
	std::vector<cv::Point> seg_cent(n_segment+1); //セグメントの重心点
	for( int i = 0; i < n_segment+1; i++ ){	//セグメントの数のループ
		int seg_num;		//各セグメント内に存在する把持点の数
		int seg_cent_sum;
		int seg_max;
		seg_cent[i].x	= 0, seg_cent[i].y = 0;
		seg_num			= 0;
		seg_cent_sum	= 0;
		//セグメントの重心を計算
		for( int jj = 0; jj < im_label.rows; jj++ ){
			for( int ii = 0; ii < im_label.cols; ii++ ){
				if( im_label.at<unsigned short>(jj, ii) == i ){
					seg_cent[i].x += ii;
					seg_cent[i].y += jj;
					seg_cent_sum++;
				}
			}
		}
		seg_cent[i].x = (int)((double)seg_cent[i].x / (double)seg_cent_sum);
		seg_cent[i].y = (int)((double)seg_cent[i].y / (double)seg_cent_sum);
	}

	FILE *fp_l;
	cv::Point text_location;
	char text_hyp[256];
	sprintf( text_hyp, "%s_AllHyp.txt", name );
	fp_l = fopen( text_hyp, "w" );
	if( fp_l == NULL ){
		fprintf( SE, "ファイル生成に失敗しました<SceneHypothesis.txt>\n" );
	}
	for( int j = 0; j < n_segment+1; j++ ){
		int hyp_count = 0;
		for( int i = 0; i < hyp.size(); i++ ){
			if( ind->chrom[i] && hyp[i].segIdx == j ){
				if( hyp_count == 0 ){
					fprintf( fp_l, "=================================================\n" );
					fprintf( fp_l, "segment:%d\n", j );

					sprintf( text,"%d", j );
					cv::putText( im_sh, text, seg_cent[j], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar( 255, 255, 0 ), 2, CV_AA);
					cv::putText( im_sh, text, seg_cent[j], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar( 0, 0, 0 ), 1, CV_AA);
				}
				if( hyp[i].method == FAST_LABEL ) fprintf( fp_l, "[%2d] ID:%2d (%3d, %3d) __FAST score: %.3lf\n", i, hyp[i].label, hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].score );
				else if( hyp[i].method == CNN_LABEL ) fprintf( fp_l, "[%2d] ID:%2d (%3d, %3d) __CNN score: %.3lf\n", i, hyp[i].label, hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].score );
				else fprintf( fp_l, "[%2d] ID:%2d (%3d, %3d) __NoLabel\n", i, hyp[i].label, hyp[i].grasp.x, hyp[i].grasp.y );
				hyp_count++;
			}
		}
	}
	fclose( fp_l );
	// 飯塚　追加　-end

	for( int i=0 ; i<ind->chrom.size() ; i++ ){
		if( ind->chrom[i] ){
			cv::circle( im_sh, hyp[i].grasp, 5, cv::Scalar( 0, 0, 255 ), -1, CV_AA ); //色の指定はBGRの順
			cv::circle( im_sh, hyp[i].grasp, 3, cv::Scalar( 255, 255, 255 ), -1, CV_AA );
			text_location = hyp[i].grasp;
			text_location.x += 25;
			text_location.y -= 25;
			sprintf( text,"Id:%d(%d)", hyp[i].label, i );
			cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar( 255, 0, 255 ), 2, CV_AA);
			cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar( 0, 0, 0 ), 1, CV_AA);
		}
	}

	text_location.x = 30;
	text_location.y = 30;
	sprintf( text,"Fitness:%.2lf", ind->fitness );
	cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 2, CV_AA);
	cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA);

	text_location.x = 40;
	text_location.y = 50;
	sprintf( text,"score_MA:%.2lf", ind->score_MA );
	cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 2, CV_AA);
	cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA);

	text_location.x = 40;
	text_location.y = 70;
	sprintf( text,"score_NL:%.2lf", ind->score_NL );
	cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 2, CV_AA);
	cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA);

	text_location.x = 40;
	text_location.y = 90;
	sprintf( text,"score_UA:%.2lf", ind->score_UA );
	cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 2, CV_AA);;
	cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA);

	text_location.x = 40;
	text_location.y = 110;
	sprintf( text,"score_P:%.2lf", ind->score_PC );
	cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 2, CV_AA);;
	cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA);

	text_location.x = 40;
	text_location.y = 130;
	sprintf( text,"score_LC:%.2lf", ind->score_LC );
	cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 2, CV_AA);;
	cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 0, 0 ), 1, CV_AA);

	//cv::imshow( "SceneHypothesis", im_sh );
	cv::imwrite( name, im_sh );
	//cv::waitKey();

}

void saveAllHypotheses( cv::Mat *im_scene, std::vector<Hyp>& hyp, individuall *ind, char *name, int n_segment, std::vector<cv::Mat>& seg_img, cv::Mat im_label ){

	cv::Mat im_sh = cv::Mat::zeros( im_scene->rows, im_scene->cols, CV_8UC3 );
	char text[FNL];
	cv::Point text_location;

	// セグメントの描画（文字とか書くときは，この後にする）
	//im_sh = im_scene->clone();
	for( int j=0 ; j<im_sh.rows ; j++ ){
		for( int i=0 ; i<im_sh.cols ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = tmp(1) = tmp(2) = im_scene->at<uchar>( j, i );
			im_sh.at<cv::Vec3b>( j, i ) = tmp;
		}
	}

	// セグメントの描画（文字とか書くときは，この後にする）
	//im_sh = im_scene->clone();
	for( int i=0 ; i<ind->chrom.size() ; i++ ){
		//if( ind->chrom[i] ){
			int t_id;
			t_id = rand()%16;
			for( int jj=0 ; jj<im_sh.rows ; jj++ ){
				for( int ii=0 ; ii<im_sh.cols ; ii++ ){
					if( seg_img[hyp[i].segIdx].at<uchar>( jj, ii ) == 255 ){
						cv::Vec3b tmp;
						tmp(2) = color16[t_id][0]; tmp(1) = color16[t_id][1]; tmp(0) = color16[t_id][2]; 
						im_sh.at<cv::Vec3b>( jj, ii ) = tmp;
					}
				}
			}
		//}
	}

	// 飯塚　追加　-start
	std::vector<cv::Point> seg_cent(n_segment+1); //セグメントの重心点
	for( int i = 0; i < n_segment+1; i++ ){	//セグメントの数のループ
		int seg_num;		//各セグメント内に存在する把持点の数
		int seg_cent_sum;
		int seg_max;
		seg_cent[i].x	= 0, seg_cent[i].y = 0;
		seg_num			= 0;
		seg_cent_sum	= 0;
		//セグメントの重心を計算
		for( int jj = 0; jj < im_label.rows; jj++ ){
			for( int ii = 0; ii < im_label.cols; ii++ ){
				if( im_label.at<unsigned short>(jj, ii) == i ){
					seg_cent[i].x += ii;
					seg_cent[i].y += jj;
					seg_cent_sum++;
				}
			}
		}
		seg_cent[i].x = (int)((double)seg_cent[i].x / (double)seg_cent_sum);
		seg_cent[i].y = (int)((double)seg_cent[i].y / (double)seg_cent_sum);
	}

	FILE *fp_l;
	char text_hyp[256];
	sprintf( text_hyp, "%s_AllHyp.txt", name );
	fp_l = fopen( text_hyp, "w" );
	if( fp_l == NULL ){
		fprintf( SE, "ファイル生成に失敗しました<AllHypothesis.txt>\n" );
	}
	for( int j = 0; j < n_segment+1; j++ ){
		int hyp_count = 0;
		for( int i = 0; i < hyp.size(); i++ ){
			if( hyp[i].segIdx == j ){
				if( hyp_count == 0 ){
					fprintf( fp_l, "=================================================\n" );
					fprintf( fp_l, "segment:%d\n", j );

					sprintf( text,"%d", j );
					cv::putText( im_sh, text, seg_cent[j], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar( 255, 255, 0 ), 2, CV_AA);
					cv::putText( im_sh, text, seg_cent[j], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar( 0, 0, 0 ), 1, CV_AA);
				}
				if( hyp[i].method == FAST_LABEL ) fprintf( fp_l, "[%2d] ID:%2d (%3d, %3d) __FAST score: %.3lf\n", i, hyp[i].label, hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].score );
				else if( hyp[i].method == CNN_LABEL ) fprintf( fp_l, "[%2d] ID:%2d (%3d, %3d) __CNN score: %.3lf\n", i, hyp[i].label, hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].score );
				else fprintf( fp_l, "[%2d] ID:%2d (%3d, %3d) __NoLabel\n", i, hyp[i].label, hyp[i].grasp.x, hyp[i].grasp.y );
				hyp_count++;
			}
		}
	}
	fclose( fp_l );
	// 飯塚　追加　-end

	for( int i=0 ; i<ind->chrom.size() ; i++ ){
		//if( ind->chrom[i] ){
			cv::circle( im_sh, hyp[i].grasp, 5, cv::Scalar( 0, 0, 255 ), -1, CV_AA ); //色の指定はBGRの順
			cv::circle( im_sh, hyp[i].grasp, 3, cv::Scalar( 255, 255, 255 ), -1, CV_AA );
			text_location = hyp[i].grasp;
			text_location.x += 25;
			text_location.y -= 25;
			sprintf( text,"Id:%d(%d)", hyp[i].label, i );
			cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar( 0, 255, 0 ), 2, CV_AA);
			cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar( 0, 0, 0 ), 1, CV_AA);
		//}
	}

	text_location.x = 30;
	text_location.y = 30;
	sprintf( text,"All hypothesis" );
	cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar( 0, 255, 0 ), 2, CV_AA);

	cv::imwrite( name, im_sh );
}

void saveHypotheses( cv::Mat *im_scene, std::vector<Hyp>& hyp, char *name ){

	cv::Mat im_sh = im_scene->clone();
	char text[FNL];
	cv::Point text_location;


	// 把持点の描画
	for( int i=0 ; i<hyp.size() ; i++ ){
		cv::circle( im_sh, hyp[i].grasp, 5, cv::Scalar( 0, 0, 255 ), -1, CV_AA ); //色の指定はBGRの順
		cv::circle( im_sh, hyp[i].grasp, 3, cv::Scalar( 255, 255, 255 ), -1, CV_AA );
		text_location = hyp[i].grasp;
		text_location.x += 25;
		text_location.y -= 25;
		sprintf( text,"Id:%d", hyp[i].label );
		cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar( 0, 255, 0 ), 2, CV_AA);
		cv::putText( im_sh, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar( 0, 0, 0 ), 1, CV_AA);
	}

	cv::imwrite( name, im_sh );
}

void saveSegmentation( std::vector<cv::Mat>& im_seg, char *name ){

	cv::Mat im_sh = cv::Mat::zeros( im_seg[0].rows, im_seg[0].cols, CV_8UC3 );
	char text[FNL];
	cv::Point text_location;

	// セグメントの描画（文字とか書くときは，この後にする）
	//im_sh = im_scene->clone();
	for( int j=0 ; j<im_sh.rows ; j++ ){
		for( int i=0 ; i<im_sh.cols ; i++ ){
			for( int k=1 ; k<im_seg.size() ; k++ ){
				if( im_seg[k].at<uchar>( j, i ) != 0 ){
					cv::Vec3b tmp;
					tmp(2) = color16[k%16][0]; tmp(1) = color16[k%16][1]; tmp(0) = color16[k%16][2]; 
					im_sh.at<cv::Vec3b>( j, i ) = tmp;
				}
			}
		}
	}

	cv::Point center;
	int area;
	for( int k=1 ; k<im_seg.size() ; k++ ){
		area = 0;
		center.x = center.y = 0;
		//セグメントの重心を計算
		for( int jj = 0; jj < im_seg[k].rows; jj++ ){
			for( int ii = 0; ii < im_seg[k].cols; ii++ ){
				if( im_seg[k].at<uchar>(jj, ii) != 0 ){
					center.x += ii;
					center.y += jj;
					area++;
				}
			}
		}
		center.x = (int)((double)center.x / (double)area);
		center.y = (int)((double)center.y / (double)area);

		char text[256];
		sprintf( text,"%d", k );
		cv::putText( im_sh, text, center, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar( 255, 255, 0 ), 4, CV_AA);
		cv::putText( im_sh, text, center, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar( 20, 20, 50 ), 2, CV_AA);

	}
	cv::imwrite( name, im_sh );
}


// 画像を上下左右にサーチすることによって，もっとも近いラベルを探索する．
// サーチ範囲は決めうちで最大300pixとした．
// -> 20160522. 画像短辺の10%にした．
void searchNeasestSegment( int x, int y, cv::Mat *im_label, int *new_label, cv::Point *new_grasp ){

	int	width, height;
	int	max_search_length;
	double search_range;

	fprintf( SE,"searchNearestSegment()\n" );

	search_range = 0.1; //画像の短辺の10%がサーチ範囲


	width = im_label->cols;
	height = im_label->rows;

	if( width < height ) max_search_length = (int)(search_range * (double)width);
	else				 max_search_length = (int)(search_range * (double)height);

	fprintf( SE,"  max_search_length: %d\n", max_search_length );

	
	bool flag;
	int	step;
	step = 1;
	flag = false;
	while( !flag ){
		//fprintf( SE,"Step:%d\n", step);
		if( x+step < width ){
			//fprintf( SE," x+step %d\n", im_label->at<short>( y, x+step ) );
			if( im_label->at<short>( y, x+step ) != 0 ){
				*new_label = im_label->at<short>( y, x+step );
				new_grasp->x = x + step;
				new_grasp->y = y;
				flag = true;
			}
		}
		if( y+step < height ){
			//fprintf( SE," y+step %d\n", im_label->at<short>( y+step, x ) );
			if( im_label->at<short>( y+step, x ) != 0 ){
				*new_label = im_label->at<short>( y+step, x );
				new_grasp->x = x;
				new_grasp->y = y + step;
				flag = true;
			}
		}
		if( 0 <= x-step ){
			//fprintf( SE," x-step %d\n", im_label->at<short>( y, x-step ) );
			if( im_label->at<short>( y, x-step ) != 0 ){
				*new_label = im_label->at<short>( y, x-step );
				new_grasp->x = x - step;
				new_grasp->y = y;
				flag = true;
			}
		}
		if( 0 <= y-step ){
			//fprintf( SE," y-step %d\n", im_label->at<short>( y-step, x ) );
			if( im_label->at<short>( y-step, x ) != 0 ){
				*new_label = im_label->at<short>( y-step, x );
				new_grasp->x = x;
				new_grasp->y = y - step;
				flag = true;
			}
		}

		if( max_search_length < step ){
			*new_label = 0;
			new_grasp->x = x;
			new_grasp->y = y;
			flag = true;
		}
		step++;

	}
}

void debug_score_MA( cv::Mat *im_label, std::vector<Hyp>& hyp, individuall *ind ){

	int	width, height;
	cv::Point text_location;
	char text[FNL];

	width = im_label->cols;
	height = im_label->rows;

	cv::Mat im_result = cv::Mat::zeros( height, width, CV_8UC3 );

	for( int j=0 ; j<height ; j++ ){
		for( int i=0 ; i<width ; i++ ){
			if( im_label->at<short>( j, i ) != 0 ){
				cv::Vec3b	color;
				int	id;
				id = 25%(int)im_label->at<short>( j, i );
				color(0) = color25[id][2];
				color(1) = color25[id][1];
				color(2) = color25[id][0];
				im_result.at<cv::Vec3b>( j, i ) = color;
			}
		}
	}

	for( int i=0 ; i<ind->chrom.size() ; i++ ){
		if( ind->chrom[i] ){	
			cv::circle( im_result, hyp[i].grasp, 5, cv::Scalar( 0, 0, 255 ), -1, CV_AA ); //色の指定はBGRの順
			cv::circle( im_result, hyp[i].grasp, 3, cv::Scalar( 255, 255, 255 ), -1, CV_AA );
			text_location = hyp[i].grasp;
			text_location.x -= 60;
			text_location.y -= 20;
			sprintf( text,"Item:%d(%d)", hyp[i].label, i );
			cv::putText( im_result, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 1, CV_AA);
		}
	}


	text_location.x = 30;
	text_location.y = 30;
	sprintf( text,"Fitness:%.3lf", ind->fitness );
	cv::putText( im_result, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 1, CV_AA);

	text_location.x = 40;
	text_location.y = 60;
	sprintf( text,"score_UA:%.3lf", ind->score_UA );
	cv::putText( im_result, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 1, CV_AA);

	text_location.x = 40;
	text_location.y = 90;
	sprintf( text,"score_P:%.3lf", ind->score_PC );
	cv::putText( im_result, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 1, CV_AA);

	//text_location.x = 40;
	//text_location.y = 120;
	//sprintf( text,"score_LR:%.3lf", ind->score_LR );
	//cv::putText( im_result, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar( 0, 255, 0 ), 2, CV_AA);

	//text_location.x = 40;
	//text_location.y = 150;
	//sprintf( text,"score_P:%.3lf", ind->score_P );
	//cv::putText( im_result, text, text_location, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar( 0, 255, 0 ), 2, CV_AA);

	//cv::imshow( "label", im_result );
	cv::imwrite( "result_score_MA.bmp", im_result );
	//cv::waitKey();
	
}

void debug_FAST_res( cv::Mat *im, std::vector<int>& ii, std::vector<int>& jj, std::vector<int>& id, std::vector<int>& n_match ){

	cv::Point location;
	char text[FNL];

	cv::Mat im_result = cv::Mat::zeros( 960, 1280, CV_8UC3 );

	for( int i=0 ; i<id.size() ; i++ ){
		cv::Vec3b	color;
		color(0) = color25[ id[i] ][2];
		color(1) = color25[ id[i] ][1];
		color(2) = color25[ id[i] ][0];

		location.x = ii[i];
		location.y = jj[i];

		cv::circle( im_result, location, 3, cv::Scalar( color(0), color(1), color(2) ), -1, CV_AA ); //色の指定はBGRの順
		location.x += 10;
		location.y -= 10;
		sprintf( text,"id:%d,%d", id[i], n_match[i] );
		cv::putText( im_result, text, location, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar( 0, 255, 0 ), 1, CV_AA);
	}


	cv::imwrite( "fast_result2.bmp", im_result );
	
}

void DeleteMultiLabel( std::vector<Hyp>& hyp, int n_segment, individuall *ind ){

	fprintf( SE,"DeleteMultiLabel()\n" );
	int n_max_item;
	n_max_item = 50;

	//縦：セグメント数，横：アイテム種類数のテーブルを作成
	//Trueになれば，そのセグメントにアイテムがあることがわかる．
	std::vector<std::vector<int>> lab_on_seg;
	lab_on_seg.resize( n_segment+1 );
	for( int i=0 ; i<lab_on_seg.size() ; i++ ){
		lab_on_seg[i].resize( n_max_item );
	}
	for( int j=0 ; j<lab_on_seg.size() ; j++ ){  //テーブルの初期化
		for( int i=0 ; i<lab_on_seg[j].size() ; i++ ){
			lab_on_seg[j][i] = 0;
		}
	}

	//セグメントid, 物体ラベルのテーブルに値を代入．存在すれば1.
	//このテーブルの横軸の合計が，ラベルの種類数になる．
	for( int i=0 ; i<ind->chrom.size() ; i++ ){
		if( ind->chrom[i] ){
			lab_on_seg[ hyp[i].segIdx ][ hyp[i].label ] = 1;
		}
	}

	int n_valid_segment;
	double	sum_score;
	n_valid_segment = 0;
	sum_score = 0.0;
	for( int j=0 ; j<lab_on_seg.size() ; j++ ){
		int kind_item, n_label;
		kind_item = n_label = 0;
		//セグメント上のアイテムidの種類数をカウント
		for( int i=0 ; i<n_max_item ; i++ ){
			kind_item += lab_on_seg[j][i];
		}

		fprintf( SE,"seg[%d] has %d label.\n", j, kind_item );
		double tmp_score;
		if( 1 < kind_item ){ //ラベルが2種類以上なら，最大のスコアを持つビットのみを生き残らせる．
			fprintf( SE," delete.\n" );
			double max_score; //最大スコア
			int max_bit;      //最大スコアのビット
			max_bit = 0;
			max_score = 0.0;
			for( int i=0 ; i<ind->chrom.size() ; i++ ){
				if( ind->chrom[i] && (hyp[i].segIdx == j) ){
					if( max_score < hyp[i].score ){
						max_score = hyp[i].score;
						max_bit = i;
					}
				}
			}
			fprintf( SE," max score: [%d] %.3lf\n", hyp[max_bit], max_score );

			//最大スコアでないビットの消去
			for( int i=0 ; i<ind->chrom.size() ; i++ ){
				if( ind->chrom[i] && (hyp[i].segIdx == j) ){
					if( i != max_bit ){
						ind->chrom[i] = false;
					}
				}
			}
		}
	}

}