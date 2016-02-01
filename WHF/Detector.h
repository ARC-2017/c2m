#pragma once

#ifndef HOUGH
#define HOUGH
#include "Decisiontree.h"
#include "makeSubset.h"
#include <windows.h>
#include <tchar.h>
#endif

typedef struct{

	Mat patch;						//�e�X�g�p�b�`
	vector<cv::Point> votePoint;			//���[�_
	cv::Point cutPoint;			//�e�X�g�p�b�`�̐؂�o���ʒu
	vector<double> likelihood;	//���[�����ޓx
}VotingPatch;


class Detector
{
private:

	int numTrees;								// �؂̐�
	int maxDepth;								// �؂̐[��
	int featureTests;							// �����ʑI����
	int thresholdTest;						// 臒l�I����
	int numClass;								// �N���X��

	vector<Node *> root;						// �m�[�h�|�C���^�^�z��
	vector<testSample> testSamples;		// �e�X�g�T���v��
	vector<testSample> TSamples;			// �e���v���[�g�T���v��
	vector<double> dist;						// �N���X�m��

	Point Centroid;							// ���[�m�[�h�̃Z���g���C�h
	vector<Point> Offset;					// ���[�m�[�h�̃I�t�Z�b�g���X�g
	vector<int> angle;						// ���[�m�[�h�̃p�b�`�̊p�x���X�g
	vector<int> plane;
	vector<int> Id;							// ���[�m�[�h�̃p�b�`��ID���X�g
	vector<int> subClass;					// �T�u�N���X

	double *likelihoodmaps3D[ POS_NUM ];						//x, y�� + �p�x��3�����ޓx�}�b�v
	double *likelihoodmap;

	size_t testsize;		//�e�X�g�p�b�`��

	//�F�����̕ۑ�
	vector< int > estimate_angles;
	vector< Point > estimate_Points;
	vector< double > estimate_Votes;
	vector< int > estimate_Planes;

	int PatchNum;
	vector<int> PatchNum2;
	vector<Mat> TPatches;

	inline Node *loadNode( ifstream & ifs );
	inline bool Traversal( Node *node, testSample &testSamp, vector<testSample> &Tsamp );
	inline bool viewMap(cv::Mat testImage, double *workPos, double *score ,int count);
	inline bool makeMap(cv::Mat testImage, vector<testSample> &Tsamp ,double *workPos, double *score, int count);//���t�@�����X�ԍ���Ԃ��悤�ɕύX�@���c

	inline void *SingleMemSet( void *input, double c, size_t num );	// double�^�̃|�C���^��memset
	
public:
	double computeError(int itemidx, cv::Mat testImage, double *workPos, double *score, int count);//���t�@�����X�ԍ���Ԃ��悤�ɕύX�@���c

	Detector(void);
	~Detector(void);
};