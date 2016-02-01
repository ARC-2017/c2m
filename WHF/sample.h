#pragma once
#include "WHFcommon.h"

using namespace cv;

const unsigned int B_LUT[BIT_NUM] = {1, 2, 4, 8, 16, 32, 64, 128};

const int BITS_COUNT_TABLE[256] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
};



typedef struct{

	unsigned int feature0_180;
	unsigned int feature180_360;
	unsigned int featureOri;
}dot_feature;

class DOT_IH
{

public:
	DOT_IH( void );
	~DOT_IH( void );

	int SetupFeature();
	int CreateGradLUT();
	int CreateIntegralHistogram(const cv::Mat &img);
	int	CompIntegralImage(int image_w, int image_h);
	int Getfeature(dot_feature* dot, int x, int y, int w_size, int h_size);
	int Normalize(dot_feature* dot);
	int SizeInit();
	
private:

	typedef struct{
		int block_num;
		int block_x;
		int block_y;
		int cell_x;
		int cell_y;
		int bin;
	}setup;

	setup setDOT[FEATURE];
	int sum_block;
	int img_w;
	double** image[ORIENTATION];
	double cell_hist[CELL_X][CELL_Y][ORIENTATION];
	double cell_center[CELL_X][CELL_Y][ORIENTATION];
	double magnitudeLUT[LUT_SIZE][LUT_SIZE];
	int gradLUT[LUT_SIZE][LUT_SIZE];
	int BinaryLUT[FEATURE];
	int BinaryLUT2[BIT_SET_TMP];
	int feature_num;
	int feature_dot;
	unsigned char* imgSource2;
	int wiStep;
};


class sample{
public:
	Mat patch;			//�����x�N�g��
	int label;			//���t�M��
	int angle;			//�p�b�`�̊p�x
	Point offset;		//�p�b�`�̃I�t�Z�b�g��
	dot_feature dot[CELL_NUM];
	int Id;				//�e���v���[�g�p�b�`�ԍ�

	bool insert( Mat _patch, int _label, Point _offset, dot_feature* _dot, int _angle, int _Id );
	bool Tinsert( Mat _patch, int _label, Point _offset, dot_feature* dot );
	bool console_out( void );
};

//�C���X�^���X�p�N���X
class instance{
public:

	int BagId;						//Bag�ԍ�
	int instanceId;				//�C���X�^���X�ԍ�
	Mat patch;						//�����x�N�g��
	int label;						//���t�M��
	int subLabel;					//�T�u�N���X�̃��x��
	int angle;						//�p�b�`�̊p�x
	int plane;						//�ʔԍ�
	Point offset;					//�p�b�`�̃I�t�Z�b�g��
	Point cutP;						//�p�b�`�̐؂�o���ʒu
	dot_feature dot[CELL_NUM];	//DOT����
	int Id;							//�e���v���[�g�p�b�`�ԍ�
	double pi;						//Bag�̃N���X�ޓx
	double pij;						//�p�b�`�̃N���X�ޓx
	double weight;					//�|�W�e�B�u�ƃl�K�e�B�u�ɑ΂���p�b�`�̏d��
	double weight_sub;			//�T�u�N���X�ɑ΂���p�b�`�̏d��

	bool insert( int _Bagid, Mat _patch, int _label, int _subLabel, int _plane, Point _offset, Point _cutP, dot_feature* _dot, 
		int _angle, int _Id, int _instanceId, double pi, double pij, double _weight );
	bool console_out( void );
};

class testSample{
public:
	Mat patch;
	Point point;
	dot_feature dot[CELL_NUM];
	
	bool insert( Mat _patch, Point _point, dot_feature* _dot );
	bool console_out( void );
	testSample(void);
	~testSample(void);
};

