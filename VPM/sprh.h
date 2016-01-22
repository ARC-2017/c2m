#ifndef INCLUDE_sprh_h_
#define INCLUDE_sprh_h_

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "common2.h"


#include "akidata.h"

typedef struct SPRH{	//SPRH

	int			dist_max, angle_max;		// �ő勗���C�ő�p�x
	int			dist_pitch, angle_pitch;	// �q�X�g�O�����̃s�b�`
	int			dist_reso, angle_reso;		// �q�X�g�O�����̉𑜓x

	int			n_sampling;					// �|�C���g�y�A�̃T���v�����O��
	cv::Mat		gr;							//Geometric relation
	int			area;						// �A�s�A�����X�̖ʐ�


}SPRH;

#endif