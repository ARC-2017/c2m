#include "Detector.h"
#include <Windows.h>
#include <vector>
#include <iterator>
#include <algorithm>
#include <math.h>

//パス
static const string output = OUTPUT_PATH;
static const string result = RESULT_PATH;

using namespace std;
int face[] = {cv::FONT_HERSHEY_SIMPLEX, cv::FONT_HERSHEY_PLAIN, cv::FONT_HERSHEY_DUPLEX, cv::FONT_HERSHEY_COMPLEX, 
              cv::FONT_HERSHEY_TRIPLEX, cv::FONT_HERSHEY_COMPLEX_SMALL, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 
              cv::FONT_HERSHEY_SCRIPT_COMPLEX, cv::FONT_ITALIC};

int true_angle = 0;

//メモリの初期化
inline void *Detector::SingleMemSet( void *input, double c, size_t num ){

	double *tmp = ( double * )input;
	const double ch = c;

	while( num-- ){

		*tmp++ = ch;
	}

	return input;
}

///
/// 把持判定、認識結果の描画
///
inline bool Detector::viewMap(cv::Mat testImage,double *workPos, double *score,int count){

	//検出結果の表示用
	cv::Mat colorImageDetection = cv::Mat::zeros( testImage.rows, testImage.cols, CV_8UC3);
	cvtColor( testImage, colorImageDetection, CV_GRAY2RGB );

	//認識位置毎に把持判定と短形描画
	int maxX, maxY;
	double maxVote = DBL_MIN;
	int maxAngle;
	double value;
	for( int n = 0; n < estimate_Points.size(); n++ ){

		value = estimate_Votes[ n ];
		if( maxVote < estimate_Votes[ n ] ){

			maxVote = value;
			maxX = estimate_Points[ n ].x;
			maxY = estimate_Points[ n ].y;
			maxAngle= estimate_angles[ n ];
		}
	}
	workPos[0]=maxX*(1/SCALE);
	workPos[1]=maxY*(1/SCALE);

	#ifdef LOGMODE//ログを残すかどうか判定
	cv::circle( colorImageDetection, cv::Point( maxX, maxY ), 10, cv::Scalar( 0, 0, 255 ), -1, CV_AA);
	//cv::circle( colorImageDetection, cv::Point( 315, 304 ), 10, cv::Scalar( 0, 0, 255 ), -1, CV_AA);
	//スコアとアングルを画像に書き出し
	char score_t[ 256 ];
	char e_angle[ 256 ];
	char filename[256];
	//sprintf( score_t, "%f", maxVote);
	//sprintf( score_t, "Grasping");
	//sprintf( score_t, "Standby");
	//sprintf( score_t, "Detection");
	cv::putText( colorImageDetection, score_t, cv::Point( 30, 30 ), face[8], 0.8, cv::Scalar(0,0,200), 2, CV_AA );
	//sprintf( e_angle, "Angle: %d", maxAngle );
	sprintf( e_angle, "Target OBJ");
	cv::putText( colorImageDetection, e_angle, cv::Point( workPos[0]+20, workPos[1]+10 ), face[8], 0.8, cv::Scalar(0,0,200), 2, CV_AA );
	//cv::putText( colorImageDetection, e_angle, cv::Point( 315+20, 304+10 ), face[8], 0.8, cv::Scalar(0,0,200), 2, CV_AA );
	sprintf( filename, "%s%result_%d.bmp", result.c_str(),count);
	cv::imwrite(filename, colorImageDetection );
	#endif

	//メモリの開放
	std::vector<int>().swap(estimate_angles);
	std::vector<Point>().swap(estimate_Points);
	std::vector<double>().swap(estimate_Votes);
	std::vector<int>().swap(estimate_Planes);

	return true;
}

///
/// 尤度マップの作成
///
inline bool Detector::makeMap(cv::Mat testImage, vector<testSample> &Tsamp ,double *workPos,double *score,int count){
	int x;
	int y;

	//テスト画像の読み込み
	testSamples.clear();
	makeSubset mkp;

	cout << "特徴抽出中..." << endl;

	testSamples = mkp.extractPatch( testImage );
	//mkp.extractPatch( testImage ,testSamples );
	
	cout << "特徴抽出完了" << endl;

	testsize = testSamples.size();
	// 尤度マップのメモリ確保と初期化
	for( int i = 0; i < POS_NUM; i++ ){
		likelihoodmaps3D[ i ]= new double [ testImage.cols * testImage.rows ];
	}
	//likelihoodmap = new double[testImage.cols * testImage.rows];
	int cnt = 0;
	//テストパッチ数まで
	int voteCount = 0;
	double value=0.0;
	int max_angle=0;
	int max_point_x=0, max_point_y=0;
	double VoteMax = DBL_MIN;

	cout << "テストパッチ数：" << testSamples.size() << endl;
	for( int j = 0; j < (int)testSamples.size(); j++ ){

		//木の本数まで
		for( int k = 0; k < numTrees; k++ ){

			//木に通しクラス確率とオフセットベクトルを取得
			Traversal( root[ k ], testSamples[j], Tsamp );

			for( int s = 0; s < PatchNum; s++){

				//x、y：投票位置
				x = testSamples[ j ].point.x + Offset[s].x ;
				y = testSamples[ j ].point.y + Offset[s].y ;
				//cout<<"x="<<x<<"y="<<y<<endl;
				if( x < 0 || x >= testImage.cols || y < 0 || y >= testImage.rows ){
					continue;
				}

				if( dist[ POS_LABEL ] > 0.5 ){

					//x軸 + y軸 + 角度の尤度マップに投票 //余分な処理をなくして、投票時に最大点を求めるように変更 村田

					//投票(ポジティブ)
					if(testImage.data[y * testImage.cols + x]!=0){//追加

							likelihoodmaps3D[ angle[ s ] ][ (y/ LEVEL) * (testImage.cols/ LEVEL) + (x/ LEVEL) ] += dist[ POS_LABEL ];
						if( VoteMax < likelihoodmaps3D[ angle[ s ] ][ (y/ LEVEL) * (testImage.cols/ LEVEL) + (x/ LEVEL) ] ){
							VoteMax = likelihoodmaps3D[ angle[ s ] ][ (y/ LEVEL) * (testImage.cols/ LEVEL) + (x/ LEVEL) ];
							max_point_x = x;
							max_point_y = y;
							max_angle = angle[ s ] ;
						}
						
						voteCount++;
					}
				}
			}
		}
	}

	double windowMax = -DBL_MAX, windowtmp;
	cout << "投票終了 voteCount=" <<voteCount<< endl;
	cout << "投票値：" << VoteMax <<"point=("<<max_point_x<<","<<max_point_y<<")"<< endl;
	//求めた最大値の座標を保存
	estimate_Points.push_back( Point( max_point_x , max_point_y  ) );
	estimate_Votes.push_back( VoteMax );
	estimate_angles.push_back( max_angle );

	//double tmp_score;

	//スコアを算出(全体に対してVoteMaxがどれだけ高いかを算出)
	//tmp_score=1.0;//(VoteMax/VoteMaxAll);//(ピーク)/(バンド幅の合計)
	//tmp_score=(( (VoteMaxAll-VoteMax) / ( (RADIUS*2-1)*(RADIUS*2-1)-1) ) / VoteMax );//(バンド幅内ピーク以外の平均)/(ピーク)
	//*score=tmp_score;
	std::cout<<"VoteMax="<<VoteMax<<" angle="<<max_angle<<" score="<<*score<<std::endl;
	
	#ifdef LOGMODE//ログを残すかどうか判定
	//尤度マップの可視化( HSV色空間 )
	cv::Mat likelihoodmap_HSV(  (testImage.rows / LEVEL) , (testImage.cols / LEVEL) , CV_8UC3, Scalar( 0, 0 ,0 ) );
	//cv::Mat likelihoodmap_HSV(  testImage.rows, testImage.cols, CV_8UC3, Scalar( 0, 0 ,0 ) );

	double max_lik = DBL_MIN, min_lik = DBL_MAX;
	for( int j = 0; j < likelihoodmap_HSV.rows ; j++ ){
		for( int i = 0; i < likelihoodmap_HSV.cols ; i++ ){

			value = likelihoodmaps3D[ max_angle ][ j * likelihoodmap_HSV.cols+ i ];
			if( max_lik < value ){

				max_lik = value;
			}
			if( min_lik > value ){

				min_lik = value;
			}
		}
	}

	double alpha = 120.0 / ( max_lik - min_lik );

	int step_lik = likelihoodmap_HSV.step;
	int channel_lik = likelihoodmap_HSV.channels();
	for( int j = 0; j < likelihoodmap_HSV.rows; j++ ){
		for( int i = 0; i < likelihoodmap_HSV.cols; i++ ){

			value = likelihoodmaps3D[ max_angle ][ j * likelihoodmap_HSV.cols + i ];

			likelihoodmap_HSV.data[j * step_lik + i * channel_lik + COLOR_B] = 120 - (int)( value * alpha );
			likelihoodmap_HSV.data[j * step_lik + i * channel_lik + COLOR_G] = 200;
			likelihoodmap_HSV.data[j * step_lik + i * channel_lik + COLOR_R] = 200;
		}
	}

	cv::cvtColor( likelihoodmap_HSV, likelihoodmap_HSV, CV_HSV2BGR );

	char HSV_t[ 256 ];
	// 20150411 保存先の変更(秋月)
	//sprintf( HSV_t, "%slikelihoodmap.bmp", RESULT_PATH );
	sprintf( HSV_t, "%s/likelihoodmap_%d.bmp", RESULT_PATH ,count);
	cv::imwrite( HSV_t, likelihoodmap_HSV );
	cout <<" 尤度マップの作成完了\n";
	#endif


	//可視化＋workPosの代入
	viewMap(testImage,workPos,score,count);

	//メモリの開放
	std::vector<testSample> ().swap(testSamples);

	// 尤度マップのメモリ開放
	for( int p = 0; p < POS_NUM; p++ ){
		delete[] likelihoodmaps3D[ p ];
	}

	//delete likelihoodmap;

	return true;
}

///
/// 決定木を走査する関数
///
inline bool Detector::Traversal( Node *node, testSample &testSamp, vector<testSample> &Tsamp ){

	if( node->nodeType == false ){
		//末端ノード
		Offset.resize(node->PatchNum);
		angle.resize(node->PatchNum);
		plane.resize(node->PatchNum);
		Id.resize(node->PatchNum);
		subClass.resize(node->PatchNum);
		if( !node->distribution.empty() ){
			dist.clear();
			dist = node->distribution;


			//オフセットベクトル、パッチの切り出し角度、ラベル面、パッチ番号を保存、パッチのサブクラス
			for(int s = 0; s < node->PatchNum; s++){
				Offset[s] = node->Offset[s];
				angle[s] = node->langle[s];
				plane[s] = node->lplane[s];
				Id[s] = node->Id[s];
				subClass[s] = node->subClass[s];
			}
			PatchNum = node->PatchNum;

		}else{
			cout << "クラス確率が保存されてません" << endl;
			cout << "depth = " << node->depth << " Type = " << node->nodeType 
				<< " Centroid " << Centroid; 
			cout << "dist 0 = " << node->distribution[ 0 ] 
				 << "dist 1 = " << node->distribution[ 1 ] << endl; 
			exit(-1);
		}
		return true;

	}else{

		//分岐ノード
		unsigned int comparison0_180[CELL_NUM];
		int sum = CELL_NUM;
		float distance = 0;

		int testSample0_180;
		int testSampleAndTemplate0_180;
		int Template0_180;

		//サンプルとテンプレートの類似度計算
		for(int cell = 0; cell < CELL_NUM; cell++){

			//comparison0_180[cell] = testSamp.dot[cell].feature0_180 & node->feature0_180[cell];
			comparison0_180[cell] = testSamp.dot[cell].feature0_180 ^ node->feature0_180[cell];
			//testSample0_180 = (int)BITS_COUNT_TABLE[testSamp.dot[cell].feature0_180];
			testSampleAndTemplate0_180 = (int)BITS_COUNT_TABLE[comparison0_180[cell]];
			//Template0_180 = (int)BITS_COUNT_TABLE[node->feature0_180[cell]];

			//無勾配のセル同士の計算は除く
			//if( testSample0_180 == 0 && Template0_180 == 0 ){

			//	sum--;
			//}

			//if( Template0_180 > 0 ){

			//	distance += ( testSampleAndTemplate0_180 ) / (float)Template0_180;
				distance +=  testSampleAndTemplate0_180;
			//}
		}
		
		//全てのセル同士が無勾配の場合は、類似度は0
		//if(sum == 0.0){

		//	distance = 0.0;
		//}else{
	
		//	distance = distance / sum;
		//}
		//std::cout<<"dis="<<distance<<std::endl;
		if(distance < node->threshold){
			return Traversal( node->left, testSamp, Tsamp );
		}else{
			return Traversal( node->right, testSamp, Tsamp );
		}
	}
}



///
/// 決定木を読み込む関数
///
inline Node *Detector::loadNode( ifstream & ifs ){
	Node *node;
	node = new Node();

	ifs	>> node->depth >> node->index >> node->nodeType;

	if( node->nodeType ){
		// 分岐ノード
		ifs >> node->threshold;
		for(int c = 0; c < CELL_NUM; c++){

			ifs	>> node->feature0_180[c];

		}
		node->left = loadNode( ifs );
		node->right = loadNode( ifs );
	}else{

		// 末端ノード
		ifs >> node->PatchNum;		//末端ノードのポジティブパッチ数


		node->distribution.resize( numClass, 0.0 );
		for( int i =  0; i < numClass; i++ ){
			ifs >> node->distribution[ i ];
		}

		node->Offset.resize(node->PatchNum);
		node->langle.resize(node->PatchNum);
		node->lplane.resize(node->PatchNum);
		node->Id.resize(node->PatchNum);
		node->subClass.resize(node->PatchNum);
		node->Weight_Pos.resize(node->PatchNum);
		node->Weight_Sub.resize(node->PatchNum);
		node->cutP.resize(node->PatchNum);
		node->Feature0_180.resize(node->PatchNum);

		for( int s = 0; s < node->PatchNum; s++){
			ifs >> node->Offset[s].x >> node->Offset[s].y >> node->langle[s] >> node->Weight_Pos[s] >> node->cutP[s].x >> node->cutP[s].y;
			node->Labels.push_back( 1 );
		}
	}
	return node;
}

///
/// 検出のメインとなる関数
///
double Detector::computeError(int itemNum, cv::Mat testImage, double *workPos, double *score,int count){//リファレンス番号を返すように変更　村田

	int counter = 0;
	char filename[ 256 ];

	//if(count==1){
	cout << "Hough Forests の読み込み開始...\n" ; 
	//ヘッダーの読み込み
	sprintf( filename, "%sforest/%02d/header.dat", output.c_str(), itemNum);
	ifstream ifs( filename );
	if(ifs.eof() || ifs.fail()){
		cout << "can not open! file name = " << filename << endl;
		//exit(1);
		return false;//20150413秋月
	}

	ifs >> numTrees >> maxDepth >> featureTests >> thresholdTest >> numClass;

	cout << "numTrees = " << numTrees << " maxDepth = " << maxDepth 
		<< " featureTests = " << featureTests
		<< " numClass = " << numClass << endl;
	//木のルートノード配列の初期化
	root.resize( numTrees );

	//各決定木の読み込み
	for( int i = 0; i < numTrees; i++ ){
		cout << i << "本目の決定木の読み込み完了" << "\r";
		sprintf( filename, "%sforest/%02d/Forest%d.dat", output.c_str(), itemNum, i );
		ifstream ifs( filename );

		if(ifs.eof() || ifs.fail()){
			cout << "can not open! file name = " << filename << endl;
			//exit(1);
			return false;//20150413秋月
		}
		root[i] = loadNode( ifs );
		cout << "i:" << i << endl;
	}
	//}
	double start, finish;
	double fps, time;
	cout << "識別開始" << endl;
	start = static_cast<double>(cv::getTickCount());
	
	printf("テスト画像からパッチを抽出中....\n");
		cout <<"尤度マップの作成...\n";

		makeMap(testImage,TSamples ,workPos,score,count);

		cout<<  " 検出完了\n";
	
	finish = static_cast<double>(cv::getTickCount());
	time = ( ( finish - start ) / getTickFrequency() );
	fps = 1 / time;
	cout << "識別完了\ntime = " << time << "sec\tfps = " << fps << endl;

	
	//treeの開放

	//for(unsigned int t=0; t<root.size(); ++t){
		//delete root[t];
		std::vector<Node *> ().swap(root);
	//}
	root.clear();
	
	std::vector<testSample> ().swap(TSamples);

	return true;
}


Detector::Detector(void)
{
}


Detector::~Detector(void)
{
}