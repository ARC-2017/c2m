#include "WHFcommon.h"
#ifndef SAMPLE
#define SAMPLE
#include "sample.h"
#endif

using namespace cv;

/*!
@class Node
@brief 分岐ノード，末端ノードを表現できるノードクラスです．
*/
class Node{
public:
	float threshold;								// threshold
	int Trand_id;									//テンプレートパッチ番号
	const instance* Template;					//テンプレート(new)
	vector<const instance *> lSamples;		//左に分岐させたサンプル集合
	vector<const instance *> rSamples;		//右に分岐させたサンプル集合
	unsigned int feature0_180[CELL_NUM];	//DOT特徴(0～180度)
	unsigned int feature180_360[CELL_NUM];	//DOT特徴(180～360度)
	unsigned int featureOri[CELL_NUM];		//DOT特徴(45度間隔の勾配情報)
	int depth;										// depth
	int index;										// node index
	int angle;										//角度
	int PatchNum;									//末端ノードに保存されているポジティブパッチ数
	int PatchNumNeg;									//末端ノードに保存されているネガティブパッチ数
	bool nodeType;									// true:splitnode false:leafnode
	vector<double> distribution;				// クラス確率を保存
	vector<double> weightDistribution;		//重みつき確率
	vector<double> weightSubDistribution;		//重みつき確率(サブクラス)
	//vector<Point> Offsetlist;				// オフセットリスト
	Point Centroid;								// 末端ノードのセントロイド

	vector<Point> Offset;						//末端ノードのオフセットリスト
	vector<int> Labels;						//末端ノードのオフセットリスト
	vector<int> langle;							//末端ノードの角度リスト
	vector<int> lplane;
	vector<int> Id;								//末端ノードのパッチのIDリスト
	vector<int> subClass;
	vector<double> Weight_Pos;					//末端ノードのポジティブサンプルの重みリスト
	vector<double> Weight_Neg;					//末端ノードのネガティブサンプルの重みリスト
	vector<double> Weight_Sub;					//末端ノードのサブクラスの重みリスト
	vector<Point>  cutP;
	vector< vector<unsigned int> > Feature0_180;	//DOT特徴(0～180度)
	vector< vector<unsigned int> > Feature180_360;	//DOT特徴(180～360度)
	vector< vector<unsigned int> > Feature0_180_Neg;	//DOT特徴(0～180度)
	vector< vector<unsigned int> > Feature180_360_Neg;	//DOT特徴(180～360度)

	//セントロイド用
	vector<Point> PointMean;
	vector<int> AngleMean;
	vector<int> PlaneMean;
	vector<int> IdMean;

	vector<const instance*> samples;			// 末端にたどり着いたポジティブサンプルの集合
	vector<instance*> allSamples;				//末端ノードにたどり着いたサンプル集合

	//追加学習用
	vector< const instance* > learnSamples;
	vector< const instance* > missSamples;

	instance *LearnSamples;
	instance *MissSamples;

	Node *right;									// right node
	Node *left;										// left node
	bool star;										//true:階層dの一番左にある分岐関数, false:それ以外

	Node(void);
};
