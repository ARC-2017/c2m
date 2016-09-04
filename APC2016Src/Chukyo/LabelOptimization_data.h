#ifndef _LabelOptimization_data_H_
#define _LabelOptimization_data_H_
//#include <vector>
//
////大学実装アルゴリズムのインクルード
//#include "common.h"
#include "akidata.h"

//仮説データ保存用の構造体
//これが 座標+識別ラベル(+α) のデータをあらわす
typedef struct Hyp{
	cv::Mat segment;         //セグメントの画像（8bit）
	std::vector<poc1> pix;   //セグメントの画素群
	int		label;           //ラベル（物体ID）
	cv::Point	center;      //セグメントの重心
	cv::Point   grasp;	     //把持位置
	int		segIdx;		     //帰属するセグメントのインデクス
	double	score;			 //ラベルの信頼性のスコア
	unsigned char method;	 //認識手法
}Hyp;


#endif