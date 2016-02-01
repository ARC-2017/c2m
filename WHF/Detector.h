#pragma once

#ifndef HOUGH
#define HOUGH
#include "Decisiontree.h"
#include "makeSubset.h"
#include <windows.h>
#include <tchar.h>
#endif

typedef struct{

	Mat patch;						//テストパッチ
	vector<cv::Point> votePoint;			//投票点
	cv::Point cutPoint;			//テストパッチの切り出し位置
	vector<double> likelihood;	//投票した尤度
}VotingPatch;


class Detector
{
private:

	int numTrees;								// 木の数
	int maxDepth;								// 木の深さ
	int featureTests;							// 特徴量選択回数
	int thresholdTest;						// 閾値選択回数
	int numClass;								// クラス数

	vector<Node *> root;						// ノードポインタ型配列
	vector<testSample> testSamples;		// テストサンプル
	vector<testSample> TSamples;			// テンプレートサンプル
	vector<double> dist;						// クラス確率

	Point Centroid;							// 末端ノードのセントロイド
	vector<Point> Offset;					// 末端ノードのオフセットリスト
	vector<int> angle;						// 末端ノードのパッチの角度リスト
	vector<int> plane;
	vector<int> Id;							// 末端ノードのパッチのIDリスト
	vector<int> subClass;					// サブクラス

	double *likelihoodmaps3D[ POS_NUM ];						//x, y軸 + 角度の3次元尤度マップ
	double *likelihoodmap;

	size_t testsize;		//テストパッチ数

	//認識情報の保存
	vector< int > estimate_angles;
	vector< Point > estimate_Points;
	vector< double > estimate_Votes;
	vector< int > estimate_Planes;

	int PatchNum;
	vector<int> PatchNum2;
	vector<Mat> TPatches;

	inline Node *loadNode( ifstream & ifs );
	inline bool Traversal( Node *node, testSample &testSamp, vector<testSample> &Tsamp );
	inline bool viewMap(cv::Mat testImage, double *workPos, double *score ,int count);
	inline bool makeMap(cv::Mat testImage, vector<testSample> &Tsamp ,double *workPos, double *score, int count);//リファレンス番号を返すように変更　村田

	inline void *SingleMemSet( void *input, double c, size_t num );	// double型のポインタのmemset
	
public:
	double computeError(int itemidx, cv::Mat testImage, double *workPos, double *score, int count);//リファレンス番号を返すように変更　村田

	Detector(void);
	~Detector(void);
};