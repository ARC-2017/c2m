#include "WHFcommon.h"
#ifndef SAMPLE
#define SAMPLE
#include "sample.h"
#endif

using namespace cv;

/*!
@class Node
@brief ����m�[�h�C���[�m�[�h��\���ł���m�[�h�N���X�ł��D
*/
class Node{
public:
	float threshold;								// threshold
	int Trand_id;									//�e���v���[�g�p�b�`�ԍ�
	const instance* Template;					//�e���v���[�g(new)
	vector<const instance *> lSamples;		//���ɕ��򂳂����T���v���W��
	vector<const instance *> rSamples;		//�E�ɕ��򂳂����T���v���W��
	unsigned int feature0_180[CELL_NUM];	//DOT����(0�`180�x)
	unsigned int feature180_360[CELL_NUM];	//DOT����(180�`360�x)
	unsigned int featureOri[CELL_NUM];		//DOT����(45�x�Ԋu�̌��z���)
	int depth;										// depth
	int index;										// node index
	int angle;										//�p�x
	int PatchNum;									//���[�m�[�h�ɕۑ�����Ă���|�W�e�B�u�p�b�`��
	int PatchNumNeg;									//���[�m�[�h�ɕۑ�����Ă���l�K�e�B�u�p�b�`��
	bool nodeType;									// true:splitnode false:leafnode
	vector<double> distribution;				// �N���X�m����ۑ�
	vector<double> weightDistribution;		//�d�݂��m��
	vector<double> weightSubDistribution;		//�d�݂��m��(�T�u�N���X)
	//vector<Point> Offsetlist;				// �I�t�Z�b�g���X�g
	Point Centroid;								// ���[�m�[�h�̃Z���g���C�h

	vector<Point> Offset;						//���[�m�[�h�̃I�t�Z�b�g���X�g
	vector<int> Labels;						//���[�m�[�h�̃I�t�Z�b�g���X�g
	vector<int> langle;							//���[�m�[�h�̊p�x���X�g
	vector<int> lplane;
	vector<int> Id;								//���[�m�[�h�̃p�b�`��ID���X�g
	vector<int> subClass;
	vector<double> Weight_Pos;					//���[�m�[�h�̃|�W�e�B�u�T���v���̏d�݃��X�g
	vector<double> Weight_Neg;					//���[�m�[�h�̃l�K�e�B�u�T���v���̏d�݃��X�g
	vector<double> Weight_Sub;					//���[�m�[�h�̃T�u�N���X�̏d�݃��X�g
	vector<Point>  cutP;
	vector< vector<unsigned int> > Feature0_180;	//DOT����(0�`180�x)
	vector< vector<unsigned int> > Feature180_360;	//DOT����(180�`360�x)
	vector< vector<unsigned int> > Feature0_180_Neg;	//DOT����(0�`180�x)
	vector< vector<unsigned int> > Feature180_360_Neg;	//DOT����(180�`360�x)

	//�Z���g���C�h�p
	vector<Point> PointMean;
	vector<int> AngleMean;
	vector<int> PlaneMean;
	vector<int> IdMean;

	vector<const instance*> samples;			// ���[�ɂ��ǂ蒅�����|�W�e�B�u�T���v���̏W��
	vector<instance*> allSamples;				//���[�m�[�h�ɂ��ǂ蒅�����T���v���W��

	//�ǉ��w�K�p
	vector< const instance* > learnSamples;
	vector< const instance* > missSamples;

	instance *LearnSamples;
	instance *MissSamples;

	Node *right;									// right node
	Node *left;										// left node
	bool star;										//true:�K�wd�̈�ԍ��ɂ��镪��֐�, false:����ȊO

	Node(void);
};
