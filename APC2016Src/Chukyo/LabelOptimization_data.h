#ifndef _LabelOptimization_data_H_
#define _LabelOptimization_data_H_
//#include <vector>
//
////��w�����A���S���Y���̃C���N���[�h
//#include "common.h"
#include "akidata.h"

//�����f�[�^�ۑ��p�̍\����
//���ꂪ ���W+���ʃ��x��(+��) �̃f�[�^������킷
typedef struct Hyp{
	cv::Mat segment;         //�Z�O�����g�̉摜�i8bit�j
	std::vector<poc1> pix;   //�Z�O�����g�̉�f�Q
	int		label;           //���x���i����ID�j
	cv::Point	center;      //�Z�O�����g�̏d�S
	cv::Point   grasp;	     //�c���ʒu
	int		segIdx;		     //�A������Z�O�����g�̃C���f�N�X
	double	score;			 //���x���̐M�����̃X�R�A
	unsigned char method;	 //�F����@
}Hyp;


#endif