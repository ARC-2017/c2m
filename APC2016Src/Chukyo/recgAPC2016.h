#pragma once
//// ��w���F������
//// ���̓f�[�^
////	**binIdx	�r�����̕���ID�f�[�^
////	*binNum		�r�����̕��̐�
////	*itemIdx	�F���Ώە��̂�ID
////	orderBin	�I�[�_�[�̂������r����ID
////	*pnt		�|�C���g�N���E�h		
////�@	*depth		�����摜
////	*color		RGB�摜
////    backgroudID ���_���D��Ȃ�΂߁C�����Ȃ琳�ʎ��_��z�肵�܂��D
////
//// �o�̓f�[�^
////    ���ʑΏە��Ɋւ�����
////	work_i		�F���Ώە���̓_��i���W
////	work_j		�F���Ώە���̓_��j���W
////	work_score	�_�i work_i, work_j �j�̐M���x
////	cluster		�F�����ʂ̃Z�O�����g���i�[����܂��D1280x960��1�����z��ł��D
////				0, 255�̓�l�摜�ł��D
////    ���ʑΏە�'�ȊO'�Ɋւ�����
////	nt_itemIdx	�F���Ώە��ȊO��ID
////	nt_i		�F���Ώە��ȊO�̓_��i���W�z��
////	nt_j		�F���Ώە��ȊO�̓_��j���W�z��
////	nt_score	�_�i nt_i, nt_j �j�̐M���x
////	nt_cluster	�F���Ώە��ȊO�̃Z�O�����g���i�[����܂��D1280x960��1�����z��ł��D
////				0, 255�̓�l�摜�ł��D
////�@�@�M���x�}�b�v
////    c_map       �e�Z�O�����g�̐M���x���i�[����܂��D1280x960��1�����z��ł��D�l�͈̔͂�[1,100]�ł��D
////				�������C�^�[�Q�b�g�A�C�e���̓��_��+100����Ă���C�ő�l��200�ɂȂ�܂��D
////    ID�}�b�v	1280x960�̈ʒu�����z��ɁC�A�C�e��ID���U���Ă���܂��D
////		

//bool RecgAPC2016( int **binIdx, int *binNum, int *itemIdx, int orderBin, double *pnt, unsigned char *depth, unsigned char *color, int backgroundID,
//				 std::vector<int>& work_i,  std::vector<int>& work_j,  std::vector<double>& work_score, unsigned char *cluster, 
//				 std::vector<int>& nt_itemIdx, std::vector<int>& nt_i, std::vector<int>& nt_j, std::vector<double>& nt_score, unsigned char *nt_cluster,
//				 unsigned char *cmap, unsigned char *id_map );
bool RecgAPC2016( int *nItemIndices, int nItemNum, int nTargetItemIdx, int nTargetBinIdx, double *pnt, unsigned char *depth, unsigned char *color, int backgroundID,
				 std::vector<int>& work_i,  std::vector<int>& work_j,  std::vector<double>& work_score, unsigned char *cluster, 
				 std::vector<int>& nt_itemIdx, std::vector<int>& nt_i, std::vector<int>& nt_j, std::vector<double>& nt_score, unsigned char *nt_cluster,
				 unsigned char *c_map, unsigned char *id_map);


// �~���\�[�g
bool CombSort( const std::vector<double>& values, std::vector<int>& Idx );