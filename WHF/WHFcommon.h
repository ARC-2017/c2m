#include <string>
#include <math.h>
#include <float.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include "includeCV.h"

//検出結果ログをとる場合のマクロ定義
#define LOGMODE

//
// DOTパラメータ
//
#define ORIENTATION 8
#define CELL_NUM (PATCH_WIDTH / CELL_SIZE) * (PATCH_HEIGHT / CELL_SIZE)
#define BIT_SET_TMP				(int)(FEATURE / ORIENTATION)
#define BIT_SET_NUM				(int)(FEATURE / BIT_NUM + 1)
#define LUT_SIZE				511													//LUTのサイズ
#define FEATURE					(CELL_NUM * ORIENTATION)		//DOT特徴量の次元数
#define CELL_X					(PATCH_WIDTH / CELL_SIZE)						//X軸のセルの数
#define CELL_Y					(PATCH_HEIGHT / CELL_SIZE)						//Y軸のセルの数

#define ANGLE					180.0													//量子化する角度
#define BIT_NUM					9													//格納するビット数
#define BIT_TH					400//0.5												//ヒストグラム2値化のしきい値

//
// 学習パラメータ
//
// Forestsパラメータ
#define	NUM_TREES				5		//木の本数
#define	MAX_DEPTH				40		//木の深さ
#define	PER_SUBSET				0.1		//サブセットの割合
// treeパラメータ
#define NUM_CLASS					2		//クラス数
#define FEATURE_TESTS			20		//テンプレート選択回数
#define TH_TESTS					10		//閾値選択回数
#define SAMPLE_MIN				100	//末端ノード作成条件
#define SPLIT_SELECT_MIN		100	//分岐関数を選択する条件

//
// プログラムの設定
//
// 色のチャンネル
#define COLOR_B					0
#define COLOR_G					1
#define COLOR_R					2

//
// サンプルデータに関するデータ
//		
//クラスラベル
#define POS_LABEL				1
#define NEG_LABEL				0
// 学習画像枚数
#define NEG_NUM			48		//ネガティブ画像枚数
#define NEG_OBJ_NUM		24		//ネガティブ画像枚数
#define POS_NUM			48		//ポジティブ画像枚数
#define POS_OBJ_NUM		1		//ポジティブ画像枚数
//#define NEG_NUM			36		//ネガティブ画像枚数
//#define NEG_OBJ_NUM		6		//ネガティブ画像枚数
//#define POS_NUM			18		//ポジティブ画像枚数
//#define POS_OBJ_NUM		2		//ポジティブ画像枚数
// パッチに占める物体領域の割合
#define NEG_Rmin		0.0	//ネガティブ用
#define NEG_Rmax		1.0
#define POS_Rmin		0.0	//把持可能用
#define POS_Rmax		1.0
#define TEST_Rmin		0.0	//テスト用
#define TEST_Rmax		1.0
// 画像スケール係数
#define SCALE			1.0
//投票空間の量子化レベル
#define LEVEL 10
//セルサイズ
#define CELL_SIZE		7
// パッチサイズ
#define PATCH_WIDTH		49
#define PATCH_HEIGHT		49
#define HX					PATCH_WIDTH / 2
#define HY					PATCH_HEIGHT / 2
// WINDOWサイズ(偶数に設定)
#define WINDOW_WIDTH		100
#define WINDOW_HEIGHT	100
// パッチのサンプリングステップ
#define SAMPLING_WSTEP			100
#define SAMPLING_HSTEP			100
#define SAMPLING_WSTEP_POS		5		//把持可能のサンプリングステップ
#define SAMPLING_HSTEP_POS		5
//テスト画像枚数
#define TEST_NUM		20
//テスト画像のサンプリングステップ
#define TEST_SS			8//4

//
// ファイルパス 
//
//HoughForestsの保存ファイルパス
//#define OUTPUT_PATH			"G:\\data\\murata\\RecgTest11\\RecgSrc\\"
#define OUTPUT_PATH			"H:\\data\\shoheik\\WHF\\WHF\\"
//結果画像の保存ファイルパス 
#define RESULT_PATH			".\\output2\\"
//
//detection のパラメータ
//
//バンド幅
#define RADIUS		1		//偶数にする
#define RADIUS2		1		//偶数にする
#define RADIUS3		1		//偶数にする
////尤度の閾値
#define LIKELIHOOD_TH		10.0//0.6
//識別のしきい値
#define DET_TH					500.0
//検出点統合のしきい値
#define DIS_TH					100.0
