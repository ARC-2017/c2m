/////////////////////////////////////////////////////////////////////////////
//
//	common.h:
//
//  Shuichi AKIZUKI
//
//	(C) 2015 ISL, Chukyo University All rights reserved.
//
//  Note:
//  VPM�CSimple�̂��߂�
//	�p�����[�^��p�X�C�f�o�b�O�I�v�V�����̐錾�D
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

//�A���S���Y���̔ԍ��iRecg�̃t���O�Ɋւ���ԋp�l�ƈ�v�j
#define METHOD_WHF	0		//Weighted Hough Forest
#define METHOD_VPM 1		//Vector Pair Matching
#define METHOD_SIMPLE 2	//Simple method (SPRH)
#define METHOD_ERROR 3	    //�G���[�̔ԍ�
#define METHOD_FAST	4		//FAST�p�ɒǉ��@15.5.10 ���c

//���O�t�@�C���f���o��
#define OUTPUT1 0 //�ׂ������O���o��
#define OUTPUT2 0 //�d�v�ȃ��O���o��
#define OUTPUT3 1 //�Œ���̃��O���o��

/*------------------------------------------------------------------------------------------*/
//���ʓI�Ȃ���

//�v���f�[�^�̉𑜓x
#define WIDTH 1280
#define HEIGHT 960

//���_�ʒu
#define VIEWPOINT_X 0.0
#define VIEWPOINT_Y 0.0
#define VIEWPOINT_Z 1.0

// �f�o�b�O�p�����摜�̃T�C�Y�C1��f������̃s�b�`
#define IMG_COLS 400
#define IMG_ROWS 400
#define PIXEL_PITCH 1.0

// �f�o�b�O�p�����摜�����Ƃ��̃I�t�Z�b�g��
#define TRANS_ZERO_X 250.0
#define TRANS_ZERO_Y 250.0
#define TRANS_ZERO_Z 850.0
/*------------------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------------------*/
//�p�X�֌W
//���f���f�[�^�̏ꏊ
// ���f���f�[�^�̓A�C�e��ID�����O�ɂȂ��Ă���f�B���N�g���ɂ܂Ƃ߂ē���Ă����D
// ���Ƃ��΁CID=4�̃A�C�e���̂Ƃ��C�u4�v�̃f�B���N�g����
// 4.pcd (�A�C�e���̓_�Q�f�[�^)
// 4.vp (�x�N�g���y�A)             ������D
#define MODEL_PATH "C:\\Temp\\Model\\"
#define SPRH_NAME "_sprh"  //�V���v���A���S���Y���̓����ʂ̖��O
#define CH_NAME "_colorHistogram"  //�V���v���A���S���Y���̓����ʂ̖��O
/*------------------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------------------*/
//�p�����[�^�֘A�i�r�����̓_�Q�̎��o���j
//
// �����̃f�[�^�� bin_size.h �Ɉړ������D
//
//#define BIN_PARAM_NAME "bin_param.txt" //�r���p�����[�^�̃t�@�C����
//#define DIST_CAMERA2BIN	-590.0 //�J��������r���ւ̐�������
//#define CAMERA_INCLINATION -10.0 //�J�����̌X���D���������}�C�i�X
//#define BIN_WIDTH 240.0 //�r���̉���

/*------------------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------------------*/
//�p�����[�^�֘A�iVPM�j

#define MODEL_DOWNSAMPLE 0.3 //���̃��f���̃_�E���T���v�����O���[�g
#define	N_OUTLIER_REMOVAL 1 //�A�E�g���C�A�����̉�

// 2�i�K�̃N���X�^�����O�̂������l
#define PITCH_VOTING_SPACE 1.5 //���[��Ԃ̑e���D�_�E���T���v�����O���[�g�̉��{�Ȃ̂�������D
#define RATE_TRANS_VOTE 0.03 //1�i�K�ڂ̂������l�D�S�̂̓��[�̂�����ʉ�%��L���Ƃ��邩����D
#define TH_NVOTE_ROT 2 //2�i�K�߂̂������l�D���̐����ȏ�̃x�N�g���y�A�ɂ���Ďx������Ă����p��������L���Ƃ���D

//���f���}�b�`���O�Ɋւ���p�����[�^
#define TH_ERROR_MULTI 2.5 //�������o���Ɏg���D���̃X�R�A�ȏ�̎p���������o�͂���D
#define TH_N_INLIER 0.0 //�p�������̃C�����C�A�������̊����ȉ��������ꍇ�C���p����D(default 0.1)

#define VP_DIR_X 0.0 //���ʌ����̖@���̐ݒ�D����Ƃ̓��ϒl�ɂ���Č덷�`�F�b�N�Ώۂ����߂�D
#define VP_DIR_Y 0.0 
#define VP_DIR_Z 1.0
#define	FRONT 0.173 //0.173 VP_DIR�Ƃ̓��ς�FRONT�ȏ�̂��̂��덷�`�F�b�N�Ώ�
#define	VALID_SCORE 0.7 //�f�o�b�O�p�D���̃X�R�A�ȏ�̎p��������ۑ�

#define N_BIN 10	//�e�[�u���̉𑜓x
#define TH_Final_VPM 0.7 //�F�����ʂ�Ԃ����ǂ����̂������l
/*------------------------------------------------------------------------------------------*/



/*------------------------------------------------------------------------------------------*/
//�p�����[�^�֘A�iSimple method(SPRH)�j

#define S_LEAF			  3.0	//Downsampling size
#define	N_NEIGHBOR		  15.0 //���̔��a���̃f�[�^�������Ė@������
#define	N_NEI_C			  15.0 //���̔��a���̃f�[�^��������Color Segmentation �p�����[�^�X�V 2015.05.27 ����
#define	TH_C			  20.0 //Segmentation���̃J���[�̂������l �p�����[�^�X�V 2015.05.27 ����
#define	CUR				  0.03 //���̋ȗ��ȉ��̃f�[�^���폜
#define VALID_SEGMENT_SIZE 0.1 //�Z�O�����g�_�����C(�S�_)�~(���̐��l)�ȏ�Ȃ�ގ��x�v�Z��L���Ƃ���
/*------------------------------------------------------------------------------------------*/



#endif