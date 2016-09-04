#ifndef _GA_H_
#define _GA_H_

#include <vector>
#include <algorithm>

#define  INITIAL 0	 //初期個体
#define  CROSSOVER 1 //交叉で生成された
#define  MUTATION 2	 //突然変異で生成された
#define  MODIFIED 3	 //遺伝子組み換え

typedef struct individuall{

	std::vector<bool> chrom;	//染色体
	double fitness;				//適応度
	bool lethal_gene;	   		//致死遺伝子のフラグ
	unsigned char born;			//生成された処理を記録

	double	score_MA;	//Multiple assign
	double	score_NL;	//Number of Labels
	double	score_UA;	//Understood Area
	double	score_PC;	//Physical conflict
	double	score_LC;	//Label Confidence

	
}individuall;

bool decrease( const individuall& left, const individuall& right );

class GA{

public:
	std::vector<individuall> ind;
	std::vector<individuall> child;

	bool crossover_occured;
	bool mutation_occured;


	int getN_ind();
	int getGeneLength();
	double getAveFitness();
	double getSumFitness();
	int getN_on_bit( int n );


	//-----------------------------------------------------------------------
	// コアの処理
	//-----------------------------------------------------------------------
	//
	//	GA全体の処理手順
	//
	//  geherateInitialIndividuals()で初期個体生成
	//  ↓GAクラス外で適応度計算
	//  calcAveFittness()で平均適応度，合計適応度を計算
	//	↓ループ開始
	//	生起確率に基づいてcrossover()で交叉
	//  ↓
	//	生起確率に基づいてmutation()で突然変異
	//  ↓
	//  子供個体が生成されたら，GAクラス外で適応度計算 
	//  ↓
	//  update()で世代交代
	//  ↓
	//  calcAveFittness()で平均適応度，合計適応度を計算
	//   ----->ループのはじめに戻る
	//

	// 初期個体の生成と初期化
	// num: 個体数，length: 遺伝子長，max_on: ONにする遺伝子の最大数
	void generateInitialIndividuals( int num, int length, int max_on );
	//交叉
	void crossover();
	//突然変異
	void mutation();
	//適応度順の並び替え
	void SortIndividuals();
	//世代交代（子供個体を入れて，ソートする）
	void update();


	//平均適応度の計算（合計適応度も更新される）
	void calcAveFitness();

	//-----------------------------------------------------------------------
	// 可視化関連・デバッグ用関数など
	//-----------------------------------------------------------------------
	//n 番目の個体を表示
	void showIndividual( int n );
	//全個体を表示
	void showAllIndividuals();
	//n 番目の子供個体を表示
	void showChildIndividual( int n );
	
	//適応度を使ったルーレットルールが正しく動くかどうかのテスト
	//たくさんの回数の親選択を実行して，選択回数を記録．
	//親の適応度と選択回数を出力．
	void Ztest1(); //->デバッグ完了．

	//適応度順のソートのテスト．
	void Ztest2(); //->デバッグ完了．

	//世代交代のテスト．
	void Ztest3(); //->デバッグ完了．

private:

	int	n_ind;   //個体数
	int	length;  //遺伝子長

	double	max_fitness; //最大適応度
	double	ave_fitness; //平均適応度
	double	sum_fitness; //合計適応度(ルーレット用)
	
	void setN_ind( int n_ind );
	void setLength( int length );
	void setSumFitness( double sum );

	// 適応度を使ったルーレットルールによる親選択
	int selectParent();
};

#endif