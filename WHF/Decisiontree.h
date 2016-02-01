#include "WHFcommon.h"
#ifndef SAMPLE
#define SAMPLE
#include "sample.h"
#endif
#include "Node.h"
#include <ctime>
#include <cmath>
#include "includeCV.h"
#include <queue>

using namespace cv;
using namespace std;

class negativeRate{

public:

	vector<double> pi;		//Bagiのネガティブ度
	vector< vector<double> > pij;		//Bagi内のインスタンスjのネガティブ度
	vector< vector<double> > wij;		//インスタンスijの重み
};
class subClass{

public:

	vector<double> pi;		//Bagiのネガティブ度
	vector< vector<double> > pij;		//Bagi内のインスタンスjのネガティブ度
	vector< vector<double> > wij;		//インスタンスijの重み
};

/*!
@class Decisiontree
@brief 決定木ひとつを表現しているクラス
*/
class Decisiontree{
private:
	Node *root;
	Node *tmpNode;
	int numTrees;
	int maxDepth;
	int featureTests;
	int thresholdTest;
	int numClass;
	int treeIndex;
	int nodeNum;
	int depthCount;
	int nodeCount;
	int maxNodeNum;

	// 分岐関数を定義
	inline bool splitFunction( const instance* &samples, const instance* &Tsamples, float &th);
	
	//ノード分岐関数の選択
	Node * createNode( Node *node, const vector<const instance *> &samples, const vector<const instance *> &Tsamples, vector<double> &il, int depth, int index );

	//決定木の構築(重み更新Ver.)
	Node * buildVerWeight( const vector<const instance *> &samples, const vector<const instance *> &Tsamples, vector<double> &il, vector<double> &il_sub, int depth, int index );

	// 情報利得を算出
	inline double computeInformationGain( std::vector<const instance *> &lSamples, std::vector<const instance *> &rSamples, vector<double> &il );

	// 情報利得を算出(サブクラス)
	inline double computeInformationGainSub( std::vector<const instance *> &lSamples, std::vector<const instance *> &rSamples, vector<double> &il );

	// 分散を算出
	inline double computeVariance( std::vector<const instance *> &lSamples, std::vector<const instance *> &rSamples );

	// 末端ノードの正規化を行う
	inline bool leafNormalization(Node *node, const vector<const sample *> &samples, vector<double> &il);

	// トラバーサルを行う
	inline bool traversalSample(Node *node, const instance* &samp, vector<double> &il);

	// 正規化を行う
	inline bool normalization(Node *node);

	// ノードを保存
	inline bool saveNode( Node *node, ofstream & ofs );

	//重みの更新
	inline bool weightUpdate( queue<Node *> &splitNode, queue<Node *> &leafNode );

	// 正規化
	inline bool weightNormalization( const vector<const instance *> &samples );

	// ノードの正規化
	bool Normalize( Node *& node );


public:

	// 学習を実行
	bool Learn( const vector<const instance *> &samples, const vector<const instance *> &Tsamples, vector<double> &il, vector<double> &il_sub );

	// 学習を実行(重み更新Ver.)
	bool LearnVerWeight( const vector<const instance *> &samples, const vector<const instance *> &Tsamples, vector<double> &il, vector<double> &il_sub );

	// 決定木に学習サンプルを入力
	bool Traversal( vector<const instance *> samples, vector<double> &il);

	// ヒストグラムを正規化
	bool NormalizeHistogram( void );

	// 木を保存
	bool save( void );

	// 木を初期化
	bool Initilizer( int _numTrees, int _maxDepth, int _featureTests, int _thresholdTest,  int _numClass, int _treeIndex );

	// コンストラクタ
	Decisiontree( int _numTrees, int _maxDepth, int _featureTests, int _thresholdTest,  int _numClass );

	// デフォルトコンストラクタ
	Decisiontree(void);

	// デストラクタ
	~Decisiontree(void);
};

