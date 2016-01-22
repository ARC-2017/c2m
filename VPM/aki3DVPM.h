/////////////////////////////////////////////////////////////////////////////
//
//	aki3DVPM.h: Hedder for 3-D Vector Pair Matching.
//
//  Shuichi AKIZUKI
//
//	(C) 2012 ISL, Chukyo University All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef INCLUDE_aki3DVPM_h_
#define INCLUDE_aki3DVPM_h_


#include <vector>
#include <list>
#include <iostream>
#include <algorithm>

typedef struct co3d{ //3D coordinate
	int		x;		
	int		y;
	int		z;
}co3d;

typedef struct aki_uchar_3d{ //3D space(unsigned char)
	int				sx;
	int				sy;
	int				sz;
	unsigned char	***img3d;
}aki_uchar_3d;

typedef struct aki_int_3d{ //3D space(int)
	int				sx;
	int				sy;
	int				sz;
	int				***img3d;
}aki_int_3d;

typedef struct aki_double_3d{ //3D space(double)
	int				sx;
	int				sy;
	int				sz;
	double			***img3d;
}aki_double_3d;

typedef struct vector_pair{
	double3		p;		//�n�_�̍��W
	double3		q1;		//�I�_�P�̍��W
	double3		q2;		//�I�_�Q�̍��W
	double3		np;		//�n�_�̖@��
	double3		nq1;	//�I�_�P�̖@��
	double3		nq2;	//�I�_�Q�̖@��
	double3		nvp;	//�x�N�g���y�A�̖@��
	double		ip_p;
	double		ip_q1;
	double		ip_q2;
	double3		vc;
	int			p_idx;
	int			q1_idx;
	int			q2_idx;
	double		observability;
	double		normalized_obs;
	double		occurrence_prob;
	double		normalized_op;
	double		distinctiveness;
}vector_pair;

typedef struct vp_hash{
	vector_pair ****vp;		//Registrated vector pair
	int			***n;		//The number of registrated vector pair
	int			sp;			//side length
	int			sq1;		//side length
	int			sq2;		//side length
	int			length;		//side length
}vp_hash;	

typedef struct VotingSpaceRot{ //Voting Space (Rotation)
	int				sx;
	int				sy;
	int				sz;
	int				nVP;
//	double			***rot_x;
//	double			***rot_y;
//	double			***rot_z;
//	double			***trans_x;
//	double			***trans_y;
//	double			***trans_z;
	int				****id;
	int				***nVote;
}VotingSpaceRot;

typedef struct VotingSpaceTrans{ //Voting Space (Trans)
	int				sx;
	int				sy;
	int				sz;
	int				nVP;
	std::vector< std::vector< std::vector< std::vector<int> > > > kind; //����ނ̓��[�����������m�邽�߃t���O�z��
	std::vector< std::vector< std::vector< std::list<int> > > > id; //�p��������ID
	std::vector< std::vector< std::vector<int> > > nVote; //����ނ̓��[�����������L�^
}VotingSpaceTrans;

#endif
