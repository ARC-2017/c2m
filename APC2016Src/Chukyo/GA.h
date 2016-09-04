#ifndef _GA_H_
#define _GA_H_

#include <vector>
#include <algorithm>

#define  INITIAL 0	 //������
#define  CROSSOVER 1 //�����Ő������ꂽ
#define  MUTATION 2	 //�ˑR�ψقŐ������ꂽ
#define  MODIFIED 3	 //��`�q�g�݊���

typedef struct individuall{

	std::vector<bool> chrom;	//���F��
	double fitness;				//�K���x
	bool lethal_gene;	   		//�v����`�q�̃t���O
	unsigned char born;			//�������ꂽ�������L�^

	double	score_MA;	//Multiple assign
	double	score_NL;	//Number of Labels
	double	score_UA;	//Understood Area
	double	score_PC;	//Physical conflict
	double	score_LC;	//Label Confidence

	
}individuall;

bool decrease( const individuall& left, const individuall& right );

class GA{

public:
	std::vector<individuall> ind;
	std::vector<individuall> child;

	bool crossover_occured;
	bool mutation_occured;


	int getN_ind();
	int getGeneLength();
	double getAveFitness();
	double getSumFitness();
	int getN_on_bit( int n );


	//-----------------------------------------------------------------------
	// �R�A�̏���
	//-----------------------------------------------------------------------
	//
	//	GA�S�̂̏����菇
	//
	//  geherateInitialIndividuals()�ŏ����̐���
	//  ��GA�N���X�O�œK���x�v�Z
	//  calcAveFittness()�ŕ��ϓK���x�C���v�K���x���v�Z
	//	�����[�v�J�n
	//	���N�m���Ɋ�Â���crossover()�Ō���
	//  ��
	//	���N�m���Ɋ�Â���mutation()�œˑR�ψ�
	//  ��
	//  �q���̂��������ꂽ��CGA�N���X�O�œK���x�v�Z 
	//  ��
	//  update()�Ő�����
	//  ��
	//  calcAveFittness()�ŕ��ϓK���x�C���v�K���x���v�Z
	//   ----->���[�v�̂͂��߂ɖ߂�
	//

	// �����̂̐����Ə�����
	// num: �̐��Clength: ��`�q���Cmax_on: ON�ɂ����`�q�̍ő吔
	void generateInitialIndividuals( int num, int length, int max_on );
	//����
	void crossover();
	//�ˑR�ψ�
	void mutation();
	//�K���x���̕��ёւ�
	void SortIndividuals();
	//������i�q���̂����āC�\�[�g����j
	void update();


	//���ϓK���x�̌v�Z�i���v�K���x���X�V�����j
	void calcAveFitness();

	//-----------------------------------------------------------------------
	// �����֘A�E�f�o�b�O�p�֐��Ȃ�
	//-----------------------------------------------------------------------
	//n �Ԗڂ̌̂�\��
	void showIndividual( int n );
	//�S�̂�\��
	void showAllIndividuals();
	//n �Ԗڂ̎q���̂�\��
	void showChildIndividual( int n );
	
	//�K���x���g�������[���b�g���[�����������������ǂ����̃e�X�g
	//��������̉񐔂̐e�I�������s���āC�I���񐔂��L�^�D
	//�e�̓K���x�ƑI���񐔂��o�́D
	void Ztest1(); //->�f�o�b�O�����D

	//�K���x���̃\�[�g�̃e�X�g�D
	void Ztest2(); //->�f�o�b�O�����D

	//������̃e�X�g�D
	void Ztest3(); //->�f�o�b�O�����D

private:

	int	n_ind;   //�̐�
	int	length;  //��`�q��

	double	max_fitness; //�ő�K���x
	double	ave_fitness; //���ϓK���x
	double	sum_fitness; //���v�K���x(���[���b�g�p)
	
	void setN_ind( int n_ind );
	void setLength( int length );
	void setSumFitness( double sum );

	// �K���x���g�������[���b�g���[���ɂ��e�I��
	int selectParent();
};

#endif