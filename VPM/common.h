/////////////////////////////////////////////////////////////////////////////
//
//	common.h:
//
//  Shuichi AKIZUKI
//
//	(C) 2015 ISL, Chukyo University All rights reserved.
//
//  Note:
//  VPM，Simpleのための
//	パラメータやパス，デバッグオプションの宣言．
//
//////////////////////////////////////////////////////////////////////////////
#ifndef INCLUDE_common_h_
#define INCLUDE_common_h_

#include "akidata.h"
#include "aki3DVPM.h"

#define FNL 2048
#define SE stderr

#define ON 1
#define OFF 0

//アルゴリズムの番号（Recgのフラグに関する返却値と一致）
#define METHOD_WHF	0		//Weighted Hough Forest
#define METHOD_VPM 1		//Vector Pair Matching
#define METHOD_SIMPLE 2	//Simple method (SPRH)
#define METHOD_ERROR 3	    //エラーの番号
#define METHOD_FAST	4		//FAST用に追加　15.5.10 村田

//ログファイル吐き出し
#define OUTPUT1 0 //細かいログを出す
#define OUTPUT2 0 //重要なログを出す
#define OUTPUT3 1 //最低限のログを出す

/*------------------------------------------------------------------------------------------*/
//共通的なもの

//計測データの解像度
#define WIDTH 1280
#define HEIGHT 960

//視点位置
#define VIEWPOINT_X 0.0
#define VIEWPOINT_Y 0.0
#define VIEWPOINT_Z 1.0

// デバッグ用距離画像のサイズ，1画素あたりのピッチ
#define IMG_COLS 400
#define IMG_ROWS 400
#define PIXEL_PITCH 1.0

// デバッグ用距離画像を作るときのオフセット量
#define TRANS_ZERO_X 250.0
#define TRANS_ZERO_Y 250.0
#define TRANS_ZERO_Z 850.0
/*------------------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------------------*/
//パス関係
//モデルデータの場所
// モデルデータはアイテムIDが名前になっているディレクトリにまとめて入れておく．
// たとえば，ID=4のアイテムのとき，「4」のディレクトリに
// 4.pcd (アイテムの点群データ)
// 4.vp (ベクトルペア)             を入れる．
#define MODEL_PATH "C:\\Temp\\Model\\"
#define SPRH_NAME "_sprh"  //シンプルアルゴリズムの特徴量の名前
#define CH_NAME "_colorHistogram"  //シンプルアルゴリズムの特徴量の名前
/*------------------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------------------*/
//パラメータ関連（ビン内の点群の取り出し）
//
// ここのデータは bin_size.h に移動した．
//
//#define BIN_PARAM_NAME "bin_param.txt" //ビンパラメータのファイル名
//#define DIST_CAMERA2BIN	-590.0 //カメラからビンへの水平距離
//#define CAMERA_INCLINATION -10.0 //カメラの傾き．下向きがマイナス
//#define BIN_WIDTH 240.0 //ビンの横幅

/*------------------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------------------*/
//パラメータ関連（VPM）

#define MODEL_DOWNSAMPLE 0.3 //物体モデルのダウンサンプリングレート
#define	N_OUTLIER_REMOVAL 1 //アウトライア除去の回数

// 2段階のクラスタリングのしきい値
#define PITCH_VOTING_SPACE 1.5 //投票空間の粗さ．ダウンサンプリングレートの何倍なのかを決定．
#define RATE_TRANS_VOTE 0.03 //1段階目のしきい値．全体の投票のうち上位何%を有効とするか決定．
#define TH_NVOTE_ROT 2 //2段階めのしきい値．この数字以上のベクトルペアによって支持されていた姿勢仮説を有効とする．

//モデルマッチングに関するパラメータ
#define TH_ERROR_MULTI 2.5 //複数検出時に使う．このスコア以上の姿勢仮説を出力する．
#define TH_N_INLIER 0.0 //姿勢仮説のインライア数がこの割合以下だった場合，棄却する．(default 0.1)

#define VP_DIR_X 0.0 //正面向きの法線の設定．これとの内積値によって誤差チェック対象を決める．
#define VP_DIR_Y 0.0 
#define VP_DIR_Z 1.0
#define	FRONT 0.173 //0.173 VP_DIRとの内積がFRONT以上のものが誤差チェック対象
#define	VALID_SCORE 0.7 //デバッグ用．このスコア以上の姿勢仮説を保存

#define N_BIN 10	//テーブルの解像度
#define TH_Final_VPM 0.7 //認識結果を返すかどうかのしきい値
/*------------------------------------------------------------------------------------------*/



/*------------------------------------------------------------------------------------------*/
//パラメータ関連（Simple method(SPRH)）

#define S_LEAF			  3.0	//Downsampling size
#define	N_NEIGHBOR		  15.0 //この半径内のデータをつかって法線推定
#define	N_NEI_C			  15.0 //この半径内のデータをつかってColor Segmentation パラメータ更新 2015.05.27 武井
#define	TH_C			  20.0 //Segmentation時のカラーのしきい値 パラメータ更新 2015.05.27 武井
#define	CUR				  0.03 //この曲率以下のデータを削除
#define VALID_SEGMENT_SIZE 0.1 //セグメント点数が，(全点)×(この数値)以上なら類似度計算を有効とする
/*------------------------------------------------------------------------------------------*/



#endif