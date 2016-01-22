/////////////////////////////////////////////////////////////////////////////
//
//	librecglib.h:
//
//  Shuichi AKIZUKI
//
//	(C) 2015 ISL, Chukyo University All rights reserved.
//
//  Note:
//	��w���F�������v���O�����̋��ʓI�Ȋ֐��Q�̐錾
//
//////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_librecglib_h_
#define INCLUDE_librecglib_h_

#include "common.h"
#include "common2.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>

//Point Cloud Library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//Aki 3D Library
#include "akidata.h"
#include "aki3DVPM.h"
#include "libaki3DVPMlib.h"
#include "libHSVlib.h"
#include "libPCLFunclib.h"
#include "libakiHPRobservabilitylib.h"
#include "sprh.h"
#include "libsprhlib.h"
#include "libakitimerlib.h"
#include "librecglib.h"

#include "../Chubu/WHFcommon.h"
#include "../Chubu/Detector.h"
#include "../Chubu/header.h"

//20150515 �H���ǋL������
//#include "../SegClass/seg_classification.h"
//20150515 �H���ǋL������

// �F�����ʂ̕\��
// method: �F����@�̃t���O
// workPos: ���̂̈ʒu�E�p��
// score: �X�R�A
void ShowRecognitionResult( int method, double *workPos, double *score );

// work position �̃R�s�[
// workPos1: �R�s�[��
// num: workPos�̗v�f��
// workPos2: �R�s�[��
void CopyWorkPos( double *workPos1, int num, double *workPos2 );

cv::Mat item_segmentation(cv::Mat input_img, int center_x, int center_y, int itemNum, int refNum );

bool RecgWeightedHoughForest(int *binNum, int *itemIdx, cv::Mat color, cv::Mat Sub, double *workPos,double *score, std::vector<int>& WHFclusterIdx );

bool rts_Score(int *itemIdx, unsigned char *color, std::vector<int>PntIdx, std::vector<int>SubIdx, std::vector<int>ClusterIdx, double *distance);

bool RecgCascadedFast(int *itemIdx, cv::Mat clr_trg, cv::Mat dph_img, std::vector<int>& FASTClusterIdx,double *workPos);


//// ��w���F������
//bool Recg( int binIdx[][4], int *binNum, int *itemIdx, double *pnt, unsigned char *depth, unsigned char *color, int * flag, 
//		   double *workPos, unsigned char *cluster );


#endif
