#ifndef _LabelOptimization_H_
#define _LabelOptimization_H_
//#include <vector>
//
////��w�����A���S���Y���̃C���N���[�h
#include "common.h"
#include "akidata.h"
#include "GA.h"
#include"LabelOptimization_data.h"

// 2�����̌������� true�Ȃ�����Cfalse�Ō����Ȃ�
// ���́F l1s, l1e ����̐����̎n�_�ƏI�_
// ���́F l2s, l2e ��������̐����̎n�_�ƏI�_
// �o�́F true�Ȃ�����Cfalse�Ō����Ȃ�
bool CheckIntersectionOfPairedLineSeg( cv::Point l1s, cv::Point l1e, cv::Point l2s, cv::Point l2e );

// ���� Multipe Assign �̃X�R�A���v�Z����֐��D�e�Z�O�����g���̃��x����ސ���]���i���Ȃ��قǂ悢�j
// ���́F hyp �����Q
// ���́F n_segment �Z�O�����g����
// ���́F ind �� 
// �o�́F score �X�R�A
void calc_score_MA( std::vector<Hyp>& hyp, int n_segment, individuall *ind, double *score );

// ���� Number of Labels �̃X�R�A���v�Z����֐��D�L���ȉ�������]���i�����قǂ悢�j
// ���́F hyp �����Q
// ���́F ind �� 
// �o�́F score �X�R�A
void calc_score_NL( std::vector<Hyp>& hyp, individuall *ind, double *score );

// ���� Label Relation �̃X�R�A���v�Z����֐��D�i�g��Ȃ��D�j
// ���́F 
// �o�́F
void calc_score_LR( std::vector<Hyp>& hyp, int n_item, individuall *ind, std::vector<std::vector<double>>& label_distance, std::vector<double>& obj_size, double *score );

// ���� Physical Consistency �̃X�R�A���v�Z����֐��D
// ���́F im_scene ���͋����摜�i�����p�Ȃ̂ŁC���ۂ̏����ł͎g���ĂȂ��D�j
// ���́F hyp �����Q
// ���́F ind �� 
// �o�́F score �X�R�A
void calc_score_PC( cv::Mat *im_scene, std::vector<Hyp>& hyp, individuall *ind, double *score );

// ���� Understood Area �̃X�R�A���v�Z����֐��D
void calc_score_UA( std::vector<Hyp>& hyp, std::vector<int>& seg_n_pix, individuall *ind, double *score );

// �����@�Z�O�����g���Ƃ̕��σX�R�A���v�Z����֐��D
void calc_score_LC( std::vector<Hyp>& hyp, int n_segment, individuall *ind, double *score );

// ���� �̂̓K���x���v�Z����֐��D ��L�������̊֐��̒l�̐��`�a�E
// ���́F ��L�̊֐��ŕK�v�ȓ��̓f�[�^���ׂāD
// �� ind �̃����o��fitness�ɃX�R�A������
void calcFitness( cv::Mat *im_scene, std::vector<Hyp>& hyp, int n_item, int n_segment, std::vector<int>& seg_n_pix, individuall *ind );

// ����
// ���́F 
// �o�́F
void showSceneHypothesis( cv::Mat *im_scene, std::vector<Hyp>& hyp, individuall *ind, char *name, int n_segment, std::vector<cv::Mat>& seg_img, cv::Mat im_label ); //�ђˁ@�ҏW

// ����
// ���́F 
// �o�́F
void saveAllHypotheses( cv::Mat *im_scene, std::vector<Hyp>& hyp, individuall *ind, char *name, int n_segment, std::vector<cv::Mat>& seg_img, cv::Mat im_label );	 //�ђˁ@�ҏW

void saveHypotheses( cv::Mat *im_scene, std::vector<Hyp>& hyp, char *name );

// ���� �Z�O�����e�[�V�������ʂ̉���
// ���́F im_seg �Z�O�����g�摜�z��Dname ���O
// �o�́F �Ȃ�
void saveSegmentation( std::vector<cv::Mat>& im_seg, char *name );
// �摜���㉺���E�ɃT�[�`���邱�Ƃɂ���āC�����Ƃ��߂����x����T������D
// �T�[�`�͈͂͌��߂����ōő�300pix�Ƃ����D
// ����
// ���́F 
// �o�́F
void searchNeasestSegment( int x, int y, cv::Mat *im_label, int *new_label, cv::Point *new_grasp );

// ����
// ���́F 
// �o�́F
void debug_score_MA( cv::Mat *im_label, std::vector<Hyp>& hyp, individuall *ind );

// ����
// ���́F 
// �o�́F
void debug_FAST_res( cv::Mat *im, std::vector<int>& ii, std::vector<int>& jj, std::vector<int>& id, std::vector<int>& n_match );

void DeleteMultiLabel( std::vector<Hyp>& hyp, int n_segment, individuall *ind );

#endif