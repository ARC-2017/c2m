#ifndef INCLUDE_sprh_h_
#define INCLUDE_sprh_h_

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "common2.h"


#include "akidata.h"

typedef struct SPRH{	//SPRH

	int			dist_max, angle_max;		// 最大距離，最大角度
	int			dist_pitch, angle_pitch;	// ヒストグラムのピッチ
	int			dist_reso, angle_reso;		// ヒストグラムの解像度

	int			n_sampling;					// ポイントペアのサンプリング数
	cv::Mat		gr;							//Geometric relation
	int			area;						// アピアランスの面積


}SPRH;

#endif