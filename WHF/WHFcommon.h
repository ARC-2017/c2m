#include <string>
#include <math.h>
#include <float.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include "includeCV.h"

//���o���ʃ��O���Ƃ�ꍇ�̃}�N����`
#define LOGMODE

//
// DOT�p�����[�^
//
#define ORIENTATION 8
#define CELL_NUM (PATCH_WIDTH / CELL_SIZE) * (PATCH_HEIGHT / CELL_SIZE)
#define BIT_SET_TMP				(int)(FEATURE / ORIENTATION)
#define BIT_SET_NUM				(int)(FEATURE / BIT_NUM + 1)
#define LUT_SIZE				511													//LUT�̃T�C�Y
#define FEATURE					(CELL_NUM * ORIENTATION)		//DOT�����ʂ̎�����
#define CELL_X					(PATCH_WIDTH / CELL_SIZE)						//X���̃Z���̐�
#define CELL_Y					(PATCH_HEIGHT / CELL_SIZE)						//Y���̃Z���̐�

#define ANGLE					180.0													//�ʎq������p�x
#define BIT_NUM					9													//�i�[����r�b�g��
#define BIT_TH					400//0.5												//�q�X�g�O����2�l���̂������l

//
// �w�K�p�����[�^
//
// Forests�p�����[�^
#define	NUM_TREES				5		//�؂̖{��
#define	MAX_DEPTH				40		//�؂̐[��
#define	PER_SUBSET				0.1		//�T�u�Z�b�g�̊���
// tree�p�����[�^
#define NUM_CLASS					2		//�N���X��
#define FEATURE_TESTS			20		//�e���v���[�g�I����
#define TH_TESTS					10		//臒l�I����
#define SAMPLE_MIN				100	//���[�m�[�h�쐬����
#define SPLIT_SELECT_MIN		100	//����֐���I���������

//
// �v���O�����̐ݒ�
//
// �F�̃`�����l��
#define COLOR_B					0
#define COLOR_G					1
#define COLOR_R					2

//
// �T���v���f�[�^�Ɋւ���f�[�^
//		
//�N���X���x��
#define POS_LABEL				1
#define NEG_LABEL				0
// �w�K�摜����
#define NEG_NUM			48		//�l�K�e�B�u�摜����
#define NEG_OBJ_NUM		24		//�l�K�e�B�u�摜����
#define POS_NUM			48		//�|�W�e�B�u�摜����
#define POS_OBJ_NUM		1		//�|�W�e�B�u�摜����
//#define NEG_NUM			36		//�l�K�e�B�u�摜����
//#define NEG_OBJ_NUM		6		//�l�K�e�B�u�摜����
//#define POS_NUM			18		//�|�W�e�B�u�摜����
//#define POS_OBJ_NUM		2		//�|�W�e�B�u�摜����
// �p�b�`�ɐ�߂镨�̗̈�̊���
#define NEG_Rmin		0.0	//�l�K�e�B�u�p
#define NEG_Rmax		1.0
#define POS_Rmin		0.0	//�c���\�p
#define POS_Rmax		1.0
#define TEST_Rmin		0.0	//�e�X�g�p
#define TEST_Rmax		1.0
// �摜�X�P�[���W��
#define SCALE			1.0
//���[��Ԃ̗ʎq�����x��
#define LEVEL 10
//�Z���T�C�Y
#define CELL_SIZE		7
// �p�b�`�T�C�Y
#define PATCH_WIDTH		49
#define PATCH_HEIGHT		49
#define HX					PATCH_WIDTH / 2
#define HY					PATCH_HEIGHT / 2
// WINDOW�T�C�Y(�����ɐݒ�)
#define WINDOW_WIDTH		100
#define WINDOW_HEIGHT	100
// �p�b�`�̃T���v�����O�X�e�b�v
#define SAMPLING_WSTEP			100
#define SAMPLING_HSTEP			100
#define SAMPLING_WSTEP_POS		5		//�c���\�̃T���v�����O�X�e�b�v
#define SAMPLING_HSTEP_POS		5
//�e�X�g�摜����
#define TEST_NUM		20
//�e�X�g�摜�̃T���v�����O�X�e�b�v
#define TEST_SS			8//4

//
// �t�@�C���p�X 
//
//HoughForests�̕ۑ��t�@�C���p�X
//#define OUTPUT_PATH			"G:\\data\\murata\\RecgTest11\\RecgSrc\\"
#define OUTPUT_PATH			"H:\\data\\shoheik\\WHF\\WHF\\"
//���ʉ摜�̕ۑ��t�@�C���p�X 
#define RESULT_PATH			".\\output2\\"
//
//detection �̃p�����[�^
//
//�o���h��
#define RADIUS		1		//�����ɂ���
#define RADIUS2		1		//�����ɂ���
#define RADIUS3		1		//�����ɂ���
////�ޓx��臒l
#define LIKELIHOOD_TH		10.0//0.6
//���ʂ̂������l
#define DET_TH					500.0
//���o�_�����̂������l
#define DIS_TH					100.0
