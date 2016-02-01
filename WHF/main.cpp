/*Hough Forests*/ 
// target APC
// Ryuei Murata 
// mryua@vision.cs.chubu.ac.jp
// パラメータはWHFcommon.hにて定義しています。
// 最終更新日 2015.4.8


#include "WHFcommon.h"
#include "Detector.h"



bool RecgWeightedHoughForest(int *itemIdx, cv::Mat color, double *workPos, double *score,int count );

int main(){
	//簡易的にポインタ変数のアドレスとbin番号とアイテム番号,入力画像を与えています。
	////////////////////////////////////////////////////////////////////////////////////

	char img_name[256];
	int BINNUM=1,ITEMIDX=13;
	double SCORE=0.0;
	int *itemIdx;
	itemIdx = &ITEMIDX;
	double workPos[2]={0.0};
	double *score;
	score =&SCORE;	
	

	for(int s=1;s<2;s++){
		sprintf(img_name,"./color.bmp");//image_pass
		cv::Mat color = cv::imread(img_name,1);

		if( !color.data ){
			cout << "\ncan not read image  " <<img_name<< endl;
		return false;
		}
	////////////////////////////////////////////////////////////////////////////////////	


	//RecgWeightedHoughForest
	RecgWeightedHoughForest(itemIdx,color,workPos,score,s);

	//std::cout<<"ワーク位置："<<workPos[0]<<","<<workPos[1]<<"　スコア："<<*score<<std::endl;
	}
}

//
bool RecgWeightedHoughForest(int *itemIdx, cv::Mat color, double *workPos,double *score ,int count){

		int bin,itemNum;
		cv::Mat testImage;
		//1チャンネルのグレーススケール画像を作成
		cv::cvtColor(color,testImage,CV_BGR2GRAY,1);
		itemNum=*itemIdx;

		Detector detector;
		//入力画像のスケールをWHFcommon.hで定義したSCALEの値に変換(チャレンジ当日のスケールにあわせて調整)
		resize(testImage, testImage, Size() , SCALE, SCALE, INTER_CUBIC);
		//検出用関数
		detector.computeError(itemNum,testImage,workPos,score,count);

		//ROG書き出しの判定
		#ifdef LOGMODE
		std::ofstream ofs( "./WHF_LOG.txt" ,std::ios::app);
		ofs<<"\titemIdx="<<itemNum<<"\tworkPos="<<workPos[0]<<","<<workPos[1]<<"\tscore="<<*score<<std::endl;
		ofs.close();
		#endif

		return true;
}