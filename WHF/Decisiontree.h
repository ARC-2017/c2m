#include "WHFcommon.h"
#ifndef SAMPLE
#define SAMPLE
#include "sample.h"
#endif
#include "Node.h"
#include <ctime>
#include <cmath>
#include "includeCV.h"
#include <queue>

using namespace cv;
using namespace std;

class negativeRate{

public:

	vector<double> pi;		//Bagi�̃l�K�e�B�u�x
	vector< vector<double> > pij;		//Bagi���̃C���X�^���Xj�̃l�K�e�B�u�x
	vector< vector<double> > wij;		//�C���X�^���Xij�̏d��
};
class subClass{

public:

	vector<double> pi;		//Bagi�̃l�K�e�B�u�x
	vector< vector<double> > pij;		//Bagi���̃C���X�^���Xj�̃l�K�e�B�u�x
	vector< vector<double> > wij;		//�C���X�^���Xij�̏d��
};

/*!
@class Decisiontree
@brief ����؂ЂƂ�\�����Ă���N���X
*/
class Decisiontree{
private:
	Node *root;
	Node *tmpNode;
	int numTrees;
	int maxDepth;
	int featureTests;
	int thresholdTest;
	int numClass;
	int treeIndex;
	int nodeNum;
	int depthCount;
	int nodeCount;
	int maxNodeNum;

	// ����֐����`
	inline bool splitFunction( const instance* &samples, const instance* &Tsamples, float &th);
	
	//�m�[�h����֐��̑I��
	Node * createNode( Node *node, const vector<const instance *> &samples, const vector<const instance *> &Tsamples, vector<double> &il, int depth, int index );

	//����؂̍\�z(�d�ݍX�VVer.)
	Node * buildVerWeight( const vector<const instance *> &samples, const vector<const instance *> &Tsamples, vector<double> &il, vector<double> &il_sub, int depth, int index );

	// ��񗘓����Z�o
	inline double computeInformationGain( std::vector<const instance *> &lSamples, std::vector<const instance *> &rSamples, vector<double> &il );

	// ��񗘓����Z�o(�T�u�N���X)
	inline double computeInformationGainSub( std::vector<const instance *> &lSamples, std::vector<const instance *> &rSamples, vector<double> &il );

	// ���U���Z�o
	inline double computeVariance( std::vector<const instance *> &lSamples, std::vector<const instance *> &rSamples );

	// ���[�m�[�h�̐��K�����s��
	inline bool leafNormalization(Node *node, const vector<const sample *> &samples, vector<double> &il);

	// �g���o�[�T�����s��
	inline bool traversalSample(Node *node, const instance* &samp, vector<double> &il);

	// ���K�����s��
	inline bool normalization(Node *node);

	// �m�[�h��ۑ�
	inline bool saveNode( Node *node, ofstream & ofs );

	//�d�݂̍X�V
	inline bool weightUpdate( queue<Node *> &splitNode, queue<Node *> &leafNode );

	// ���K��
	inline bool weightNormalization( const vector<const instance *> &samples );

	// �m�[�h�̐��K��
	bool Normalize( Node *& node );


public:

	// �w�K�����s
	bool Learn( const vector<const instance *> &samples, const vector<const instance *> &Tsamples, vector<double> &il, vector<double> &il_sub );

	// �w�K�����s(�d�ݍX�VVer.)
	bool LearnVerWeight( const vector<const instance *> &samples, const vector<const instance *> &Tsamples, vector<double> &il, vector<double> &il_sub );

	// ����؂Ɋw�K�T���v�������
	bool Traversal( vector<const instance *> samples, vector<double> &il);

	// �q�X�g�O�����𐳋K��
	bool NormalizeHistogram( void );

	// �؂�ۑ�
	bool save( void );

	// �؂�������
	bool Initilizer( int _numTrees, int _maxDepth, int _featureTests, int _thresholdTest,  int _numClass, int _treeIndex );

	// �R���X�g���N�^
	Decisiontree( int _numTrees, int _maxDepth, int _featureTests, int _thresholdTest,  int _numClass );

	// �f�t�H���g�R���X�g���N�^
	Decisiontree(void);

	// �f�X�g���N�^
	~Decisiontree(void);
};

