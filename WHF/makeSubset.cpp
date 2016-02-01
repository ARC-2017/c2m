#include "makeSubset.h"

using namespace std;
DOT_IH DOT;
bool feature[FEATURE];

//テスト画像からパッチを切り出し
vector<testSample> makeSubset::extractPatch( Mat &testImage ){
	vector<testSample> testPaches;
	testSample sampletmp;
	dot_feature Testdot[CELL_NUM];
	int patchNum;
	int im_clos;
	int im_rows;
	//cout << "テスト用パッチ抽出中....\r";
		
		im_clos = testImage.cols;
		im_rows = testImage.rows;

		patchNum = 0;

		DOT.SetupFeature();		//初期化
		DOT.CreateIntegralHistogram(testImage);	//Integral Histogramの作成

		cv::Mat clone;
		//テスト画像からパッチを切り出しDOT特徴を抽出
		for( int y = 0; ( y + PATCH_HEIGHT ) <= im_rows; y += TEST_SS ){
			for( int x = 0; ( x + PATCH_WIDTH ) <= im_clos; x += TEST_SS ){

				//if(testImage.at<int>(x + PATCH_WIDTH / 2 , y + PATCH_HEIGHT / 2)!=0){
				//if(testImage.at<int>(x , y)!=0){
				//if(testImage.data[y*testImage.step+x]!=0){
				Mat tmpPatch( testImage, Rect( x, y, PATCH_WIDTH, PATCH_HEIGHT ) );


				//DOT特徴の抽出
				DOT.Getfeature(Testdot, x, y, PATCH_WIDTH, PATCH_HEIGHT);	//特徴量抽出

				//testSampleに格納
				sampletmp.insert( tmpPatch, Point( x + PATCH_WIDTH / 2 , y + PATCH_HEIGHT / 2 ), Testdot );
				
				//パッチデータをtestSample配列に格納
				testPaches.push_back( sampletmp );

				//printf("パッチ数:%d\r", patchNum);
				patchNum++;
				//}	//rateIFend
				//}
			}
		}
		DOT.SizeInit();			//領域開放
		sampletmp;

	return testPaches;
}


//
makeSubset::makeSubset(void)
{
}


makeSubset::~makeSubset(void)
{
}
