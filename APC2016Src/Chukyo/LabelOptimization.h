#ifndef _LabelOptimization_H_
#define _LabelOptimization_H_
//#include <vector>
//
////大学実装アルゴリズムのインクルード
#include "common.h"
#include "akidata.h"
#include "GA.h"
#include"LabelOptimization_data.h"

// 2線分の交差判定 trueなら交差，falseで交差なし
// 入力： l1s, l1e 一方の線分の始点と終点
// 入力： l2s, l2e もう一方の線分の始点と終点
// 出力： trueなら交差，falseで交差なし
bool CheckIntersectionOfPairedLineSeg( cv::Point l1s, cv::Point l1e, cv::Point l2s, cv::Point l2e );

// 説明 Multipe Assign のスコアを計算する関数．各セグメント内のラベル種類数を評価（少ないほどよい）
// 入力： hyp 仮説群
// 入力： n_segment セグメント総数
// 入力： ind 個体 
// 出力： score スコア
void calc_score_MA( std::vector<Hyp>& hyp, int n_segment, individuall *ind, double *score );

// 説明 Number of Labels のスコアを計算する関数．有効な仮説数を評価（多いほどよい）
// 入力： hyp 仮説群
// 入力： ind 個体 
// 出力： score スコア
void calc_score_NL( std::vector<Hyp>& hyp, individuall *ind, double *score );

// 説明 Label Relation のスコアを計算する関数．（使わない．）
// 入力： 
// 出力：
void calc_score_LR( std::vector<Hyp>& hyp, int n_item, individuall *ind, std::vector<std::vector<double>>& label_distance, std::vector<double>& obj_size, double *score );

// 説明 Physical Consistency のスコアを計算する関数．
// 入力： im_scene 入力距離画像（可視化用なので，実際の処理では使ってない．）
// 入力： hyp 仮説群
// 入力： ind 個体 
// 出力： score スコア
void calc_score_PC( cv::Mat *im_scene, std::vector<Hyp>& hyp, individuall *ind, double *score );

// 説明 Understood Area のスコアを計算する関数．
void calc_score_UA( std::vector<Hyp>& hyp, std::vector<int>& seg_n_pix, individuall *ind, double *score );

// 説明　セグメントごとの平均スコアを計算する関数．
void calc_score_LC( std::vector<Hyp>& hyp, int n_segment, individuall *ind, double *score );

// 説明 個体の適応度を計算する関数． 上記いくつかの関数の値の線形和・
// 入力： 上記の関数で必要な入力データすべて．
// 個体 ind のメンバのfitnessにスコアが入る
void calcFitness( cv::Mat *im_scene, std::vector<Hyp>& hyp, int n_item, int n_segment, std::vector<int>& seg_n_pix, individuall *ind );

// 説明
// 入力： 
// 出力：
void showSceneHypothesis( cv::Mat *im_scene, std::vector<Hyp>& hyp, individuall *ind, char *name, int n_segment, std::vector<cv::Mat>& seg_img, cv::Mat im_label ); //飯塚　編集

// 説明
// 入力： 
// 出力：
void saveAllHypotheses( cv::Mat *im_scene, std::vector<Hyp>& hyp, individuall *ind, char *name, int n_segment, std::vector<cv::Mat>& seg_img, cv::Mat im_label );	 //飯塚　編集

void saveHypotheses( cv::Mat *im_scene, std::vector<Hyp>& hyp, char *name );

// 説明 セグメンテーション結果の可視化
// 入力： im_seg セグメント画像配列．name 名前
// 出力： なし
void saveSegmentation( std::vector<cv::Mat>& im_seg, char *name );
// 画像を上下左右にサーチすることによって，もっとも近いラベルを探索する．
// サーチ範囲は決めうちで最大300pixとした．
// 説明
// 入力： 
// 出力：
void searchNeasestSegment( int x, int y, cv::Mat *im_label, int *new_label, cv::Point *new_grasp );

// 説明
// 入力： 
// 出力：
void debug_score_MA( cv::Mat *im_label, std::vector<Hyp>& hyp, individuall *ind );

// 説明
// 入力： 
// 出力：
void debug_FAST_res( cv::Mat *im, std::vector<int>& ii, std::vector<int>& jj, std::vector<int>& id, std::vector<int>& n_match );

void DeleteMultiLabel( std::vector<Hyp>& hyp, int n_segment, individuall *ind );

#endif