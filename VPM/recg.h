#pragma once
// ��w���F������
// �o�̓f�[�^�̐����D
// fla:     �F�������A���S���Y���̔ԍ����ԋp����܂��D���g�p�H
// workPos: ���̈ʒu�ł��D�摜���W�� ( workPos[0], workPos[1] ) 
// cluster: ���̂ɑ����鋗���f�[�^���i�[����܂��D1280x960��1�����z��ł��D
//          �����f�[�^�̂Ȃ������̉�f�l��0�ł��D
bool Recg( int **binIdx, int *binNum, int *itemIdx, double *pnt, unsigned char *depth, unsigned char *color, int *flag, 
		   double *workPos, unsigned char *cluster );
// APC 20140414 �H���ǋL�@������