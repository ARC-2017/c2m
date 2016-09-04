///////////////////////////////////////////////////////////////////////////
//
//  Label Optimization
//
//  Shuichi AKIZUKI
//
//  (C) 2016 ISL, Chukyo University All rights reserved.
//
//  Note:
//
//  2016.03.29
//  Rev.A
//  2016.04.15
//	Score_UA�����ǂ��ď������Ԃ��������D
//  �摜�����T�C�Y���āC�������������D�{�ԗp�v���O�����Ɏ�������Ƃ��́C
//  �c���_���̈ʒu�����T�C�Y�������ƂɋC�����邱�ƁD
//	���T�C�Y�����{���������ɂ́uRESIZE�v�Ə����Ă������D
//	2016.04.24
//  main2.cpp����́C
//  [Segmentation] -> [GFE] -> [CNN]  
//  �̌��ʂ�ǂݍ���ŁC�����������Ȃ����߂̃\�[�X�t�@�C���Ƃ���D
//  �����f�[�^�̃t�H�[�}�b�g���ς�������Ƃɒ��ӂ��邱�ƁD
//  2016.04.26
//  main.cpp�Œ����������̃R�[�h�������Ă����D
//  2016.05.18
//  �����p�Ɋ֐�����i�߂��DPCL�͂���Ȃ��̂ŁC�����D
//  2016.05.18
//  LabelOptimization���֐�recgAPC2016()�ɍ��ς����D
//  ASJ��������
//  #include "use_OpenCV.h" ���R�����g�A�E�g
//
//  
//  2016.06.23
//  �ύX�_1.
//  nTargetItemIdx = 0 �̂Ƃ��̏�����ǉ��D
//  ���̏ꍇ�͍ł��M���x�̍�������ID�ɂ��ׂẴZ�O�����g�����������ďo��
//  
//
///////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include "common.h"
#include "Labeling.h"
//#include "use_OpenCV.h"
#include "ColorTableRGB2.h"
#include "GA.h"
#include "LabelOptimization.h"
//�ђ˃Z�O�����e�[�V����
#include "color_segmentation.h"
#include "../Chubu/run_cascadedFAST.h"

//CNN�֘A�̃C���N���[�h
#include "../Chubu/run_cnn.h"

#include "../FGE/run_FGE.h"



//���u����ł̔F���̃e�X�g�p�i�����I��nTargetItemIdx = 0�ɂ��܂��j
//#define DBG_KARIOKI_TEST
//����̉摜��ǂݍ��ނ��߂̃t���O
//#define DBG_IMG_LOAD



//������common�w�b�_�ɏ����������e���L�q�i�f�[�^�f�B���N�g���Ȃǁj
#define PARAM_DIR		"C:\\Temp\\RecgAPC2016Param\\"		/* �p�����[�^�f�B���N�g���� */
#define PARAM_NAME		"param.txt"							/* �p�����[�^�t�@�C���� */

void calcNormalMap_recg( cv::Mat *InImg, int PCASize, int ResizeRate, int BackGroundID, unsigned char range_th, float noise_theta_th, bool log_img, cv::Mat *top, cv::Mat *front, cv::Mat *side );


//20160519 �\�[�g�p
bool CombSort( const std::vector<double>& values, std::vector<int>& Idx ){

	static int	n, Ndata;
	int			gap, swap_int;
	int			tmp;

	Ndata = values.size();
	if( Ndata != Idx.size() ){
		fprintf( stderr,"Error! Size of vp and Idx is different.\n" );
		fprintf( stderr,"		Size of values : %d\n", values.size() );
		fprintf( stderr,"		Size of Idx: %d\n", Idx.size() );
		return false;
	}

	gap = Ndata;
	swap_int = 1;
	while( gap>1 || swap_int ) {
		gap = (int)((double)gap/1.3) ;
		if(gap == 9 || gap == 10) gap = 11 ;
		swap_int = 0;
		for( n=0; n<(Ndata-gap) ; n++ ){
			if( values[Idx[n]] < values[Idx[n+gap]] ){
				tmp = Idx[n]; Idx[n] = Idx[n+gap]; Idx[n+gap] = tmp;
				swap_int = 1;
			}
		}
	}

	return true;
}


//bool RecgAPC2016( int **binIdx, int *binNum, int *itemIdx, int orderBin, double *pnt, unsigned char *depth, unsigned char *color, int backgroundID,
//				 std::vector<int>& work_i,  std::vector<int>& work_j,  std::vector<double>& work_score, unsigned char *cluster, 
//				 std::vector<int>& nt_itemIdx, std::vector<int>& nt_i, std::vector<int>& nt_j, std::vector<double>& nt_score, unsigned char *nt_cluster,
//				 unsigned char *c_map, unsigned char *id_map ){
bool RecgAPC2016( int *nItemIndices, int nItemNum, int nTargetItemIdx, int nTargetBinIdx, double *pnt, unsigned char *depth, unsigned char *color, int backgroundID,
				 std::vector<int>& work_i,  std::vector<int>& work_j,  std::vector<double>& work_score, unsigned char *cluster, 
				 std::vector<int>& nt_itemIdx, std::vector<int>& nt_i, std::vector<int>& nt_j, std::vector<double>& nt_score, unsigned char *nt_cluster,
				 unsigned char *c_map, unsigned char *id_map ){

//���u����ł̔F���悤�I�v�V�����D�{�Ԃł�OFF�ɂ��邱��					 
#ifdef DBG_KARIOKI_TEST
	nTargetItemIdx = 0;
#endif

	char		im_color_fname[FNL], im_depth_fname[FNL], pnt_fname[FNL], paramfname[FNL], hyp_fname[FNL], fast_hyp_fname[FNL];
	char		artfname[FNL];
	double		time_func_start, time_func_end, time1, time2;

	time_func_start = static_cast<double>(cv::getTickCount());
	fprintf( SE,"RecgAPC2016 start\n" );
	//parameter of GA
	int			population; 
	int			n_on;	// # of ON bit of initial individuals
	int			n_gen;  // # of max generation
	int			n_stable_gen;  // limit of stable generation

	// ���s���I�v�V������
	bool		use_artifitial, use_viewer, log, log2, log_img, log_img2;

	//bin parameter
	int			n_item;
	int			viewpoint; //backgroundID���画�f���鎋�_���D
						   //backgroundID�������̂���Ȃ�΂߁C�����Ȃ琳�ʁD

	//�e�A���S���Y���g�p�Ɋւ���ON/OFF����t���O
	bool		use_CNN_DATA, use_FAST_DATA;
	bool		use_CNN, use_FAST;
	bool		use_SP_SEG, use_TEX_SEG, use_NORMAL_SEG;
	int			segMethod; //0.�f�v�X 1.�f�v�X�{�e�N�X�`�� 2.�@������
	bool		use_SegCenter;

	//�摜�����p�����[�^
	int			n_closing_cluster;	//�o�͂̃N���X�^�f�[�^�Ɋ|����N���[�W���O�̉�
	int			min_segment_size;   //�o�͂���ŏI�Z�O�����g�T�C�Y
	int			max_grasp_point;   //CNN�Ŏ��ʂ���c���_�̍ő吔
	int			min_grasp_point;   //CNN�Ŏ��ʂ���c���_�̍ŏ���
	int			grasp_step;			//�c���_�̃X�e�b�v��

	// Default parameters.
	strcpy( im_color_fname,"color.bmp" );
	strcpy( im_depth_fname,"depth.bmp.bmp" );
	strcpy( pnt_fname,"point3d.pnt" );
	strcpy( hyp_fname,"hyp.txt" );
	strcpy( paramfname,"param.txt" );
	strcpy( artfname,"art_gene.txt" );

	population = 100;
	n_on = 5;
	n_gen = 1000;
	n_stable_gen = 100;
	use_artifitial = false;
	use_viewer = false;
	use_CNN_DATA = use_FAST_DATA = false;
	use_SP_SEG = use_TEX_SEG = use_NORMAL_SEG = false;
	use_CNN = use_FAST = false;
	log = log2 = log_img = log_img2 = false;
	use_SegCenter = false;
	n_item = nItemNum;
	n_closing_cluster = 2;
	min_segment_size = 2000;
	max_grasp_point = 20;
	min_grasp_point = 3;
	grasp_step = 50000;
	segMethod = 0;
	//������������΂߁C��Ȃ琳��
	// -> �΂߂Ȃ�viewpoint = 1, ���ʂȂ�viewpoint = 2 
	if( backgroundID%2 ) viewpoint = 2;
	else				 viewpoint = 1;


	FILE		*fp;
	char		funcname[256];
	double		deg2rad, rad2deg;

	deg2rad = M_PI/180.0;
	rad2deg = 180.0/M_PI;

	//strcpy( funcname,"Recg()" );
	if ((fp = fopen("log_Recg.txt", "w")) == NULL) {
		fprintf( SE, "!!Error!! %s\n", funcname );
		fprintf( SE, "  ���O�t�@�C���̏����o�����ł��܂���D\n");
		return false;
	}


	// Bin �̏��̕\��
	fprintf( fp,"viewpoint:%d\n", viewpoint );
	fprintf( fp,"backgroundID:%d\n", backgroundID );
	fprintf( fp,"nItemNum:\n");
	for( int i=0 ; i<nItemNum ; i++ ){
		fprintf( fp,"%d ", nItemIndices[i] );
	}
	fprintf( fp,"\n" );
	fprintf( fp,"nItemNum: %d\n", nItemNum );
	fprintf( fp,"nTargetItemIdx: %d\n", nTargetItemIdx );
	fclose( fp );



	// [0100] �f�[�^����
	// [0101] �����摜����
	cv::Mat d_img;
	d_img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	for( int i=0 ; i<WIDTH*HEIGHT; i++ ){
		d_img.at<uchar>( i / WIDTH, i % WIDTH ) = depth[i];
	}


	// [0102] RGB�摜����
	cv::Mat color_img; // �V�[���摜�iRGB�j
	color_img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
	cv::Vec3b tmp;
	for( int i=0 ; i<WIDTH*HEIGHT ; i++ ){
		tmp(0) = color[ 3*i ];
		tmp(1) = color[ 3*i+1 ];
		tmp(2) = color[ 3*i+2 ];
		color_img.at<cv::Vec3b>( i / WIDTH, i % WIDTH ) = tmp;
	}

//2016.06.24 �H�� �摜��ǂݍ��ނ��߂̃t���O
//bin�̒��ɕۑ������d_img.bmp��color_img.bmp�ōČ��������邽��
#ifdef DBG_IMG_LOAD
	cv::Mat d_img2 = cv::imread( "C:\\Temp\\DBG_IMG\\d_img.bmp", 1 );
	cv::Mat c_img2 = cv::imread( "C:\\Temp\\DBG_IMG\\color_img.bmp", 1 );
	cv::Mat d_img22;
	cv::cvtColor( d_img2, d_img22, CV_RGB2GRAY );
	d_img = d_img22.clone();
	color_img = c_img2.clone();
#endif

	//�ǂݍ��񂾉摜�̕\��
	if( use_viewer ){
		cv::imshow( "d_img", d_img );
		cv::imshow( "color_img", color_img );
		cv::waitKey();
	}


	//[0103] �p�����[�^�t�@�C���ǂݍ���
	FILE	*fp_param;
	char	line[256];
	char	param_name[256];
	char	value[256];
	sprintf( paramfname,"%s%s", PARAM_DIR, PARAM_NAME );
	if( (fp_param = fopen( paramfname, "r" )) == NULL ){
		fprintf( SE,"!!Error!! Module:[0103]\n" );
		fprintf( SE,"  %s could not be read.\n", paramfname );
		return false;
	}else{
		while( fgets(line, 256, fp_param) != NULL){
			if( (line[0] == '#') || (!strcmp( line, "\n" )) ){
				//# The line starting '#' and 'Return' is skipped.
			}else{
				sscanf(line, "%s %s\n", &param_name, &value);  //Split 'line' into 'param_name' and 'value'

				if( !strcmp( param_name, "USE_ARTIFITIAL_GENE" ) ){
					use_artifitial = true;
					strcpy( artfname, value );
				}
				if( !strcmp( param_name, "USE_VIEWER" ) ){
					use_viewer = true;
				}

				// Parameter of GA
				if( !strcmp( param_name, "POPULATION" ) ) population = atoi( value );
				if( !strcmp( param_name, "N_ON" ) ) n_on = atoi( value );
				if( !strcmp( param_name, "N_GENERATION" ) ) n_gen = atoi( value );
				if( !strcmp( param_name, "N_STABLE_GENERATION" ) ) n_stable_gen = atoi( value );

				// Input files
				if( !strcmp( param_name, "IMG_COLOR_NAME" ) ) strcpy( im_color_fname, value );
				if( !strcmp( param_name, "IMG_DEPTH_NAME" ) ) strcpy( im_depth_fname, value );
				if( !strcmp( param_name, "PNT_NAME" ) ) strcpy( pnt_fname, value );
				if( !strcmp( param_name, "CNN_HYP_NAME" ) ) strcpy( hyp_fname, value );
				if( !strcmp( param_name, "FAST_HYP_NAME" ) ) strcpy( fast_hyp_fname, value );

				// �A���S���Y����ON/OFF����t���O
				if( !strcmp( param_name, "USE_CNN_DATA" ) ) use_CNN_DATA = true;
				if( !strcmp( param_name, "USE_FAST_DATA" ) ) use_FAST_DATA = true;
				if( !strcmp( param_name, "USE_CNN" ) ) use_CNN = true;
				if( !strcmp( param_name, "USE_FAST" ) ) use_FAST = true;
				if( !strcmp( param_name, "USE_SP_SEG" ) ) use_SP_SEG = true;
				if( !strcmp( param_name, "USE_TEX_SEG" ) ){
					use_TEX_SEG = true;
					segMethod = 1;
				}
				if( !strcmp( param_name, "USE_NORMAL_SEG" ) ){
					use_NORMAL_SEG = true;
					segMethod = 2;
				}
				if( !strcmp( param_name, "USE_SEG_CENTER" ) ) use_SegCenter = true;
				if( !strcmp( param_name, "MAX_CNN_POINT" ) ) max_grasp_point = atoi( value );
				if( !strcmp( param_name, "MIN_CNN_POINT" ) ) min_grasp_point = atoi( value );

				if( !strcmp( param_name, "LOG" ) ) log = true;
				if( !strcmp( param_name, "LOG2" ) ) log2 = true;
				if( !strcmp( param_name, "LOG_IMG" ) ) log_img = true;
				if( !strcmp( param_name, "LOG_IMG2" ) ) log_img2 = true;

				//�摜�����p�����[�^
				if( !strcmp( param_name, "N_CLOSING" ) ) n_closing_cluster = atoi( value );
				if( !strcmp( param_name, "MIN_SEGMENT_SIZE" ) ) min_segment_size = atoi( value );
				if( !strcmp( param_name, "MAX_GRASP_POINT" ) ) max_grasp_point = atoi( value );
				if( !strcmp( param_name, "MIN_GRASP_POINT" ) ) min_grasp_point = atoi( value );
				if( !strcmp( param_name, "GRASP_STEP" ) ) grasp_step = atoi( value );
			}
		}
		fclose( fp_param );
	}

	fprintf( SE,"===============Parameter===============\n");
	if( use_artifitial )   fprintf( SE,"Use artifitial gene: %s\n", artfname );
	if( use_viewer )   fprintf( SE,"Use image viewer\n" );
	if( use_CNN_DATA )   fprintf( SE,"CNN�̃_�~�[�f�[�^���g���܂��D\n" );
	if( use_FAST_DATA )   fprintf( SE,"FAST�̃_�~�[�f�[�^���g���܂��D\n" );
	if( use_CNN )   fprintf( SE,"CNN�����삵�܂��D\n" );
	if( use_FAST )   fprintf( SE,"FAST�����삵�܂��D\n" );
	if( use_TEX_SEG )       fprintf( SE,"�e�N�X�`���Z�O�����e�[�V���������삵�܂��D\n" );
	else if( use_SP_SEG )   fprintf( SE,"Super pixel�Z�O�����e�[�V���������삵�܂��D\n" );
	else if( use_NORMAL_SEG )   fprintf( SE,"�@�������x�[�X�Z�O�����e�[�V���������삵�܂��D\n" );
	else                    fprintf( SE,"Depth �Z�O�����e�[�V�����݂̂����삵�܂��D\n" );
	if( log )   fprintf( SE,"Save log images\n" );

	if( use_SegCenter )   fprintf( SE,"�Z�O�����g�d�S��c���_�Ƃ��܂��D\n" );
	fprintf( SE," Max grasp point: %d\n", max_grasp_point );
	fprintf( SE," Min grasp point: %d\n", min_grasp_point );
	fprintf( SE," ���ʓ_�̃X�e�b�v��: %d\n", grasp_step );

	fprintf( SE,"\n");
	fprintf( SE,"Genetic Algorithm\n");
	fprintf( SE," Population: %d\n", population );
	fprintf( SE," # of ON bit: %d\n", n_on );
	fprintf( SE," # of max generation: %d\n", n_gen );
	fprintf( SE," Limit of stable generation: %d\n", n_stable_gen );
	fprintf( SE,"\n");
	fprintf( SE,"Bin parameter\n");
	fprintf( SE," Number of items: %d\n", n_item );
	if( viewpoint == 1 )      fprintf( SE," ���_: �΂�\n" );
	else if( viewpoint == 2 ) fprintf( SE," ���_: ����\n" );
	fprintf( SE,"Param Imaging\n");
	fprintf( SE," N closing %d\n", n_closing_cluster );
	fprintf( SE," Min Segment size%d\n", min_segment_size );
	fprintf( SE,"===============Parameter===============\n");

	//���̊֐��̃��O�t�@�C��
	FILE *fp_logRecg;
	if( log ){
		if( (fp_logRecg = fopen( "log_recgAPC.txt", "w" )) == NULL ){
			fprintf( stderr,"File open error. recgAPC2016()\n" );
			return false;
		}
	}

	if( log_img ){
		cv::imwrite( "d_img.bmp", d_img );
		cv::imwrite( "color_img.bmp", color_img );
	}

	//[0103] �摜�̃T�C�Y�ϊ�
	// �����摜��24bit�������̂ŃO���[�X�P�[���ɕϊ�
	if( log ){
		fprintf( fp_logRecg,"�摜�̃T�C�Y�ϊ�\n" );
		fprintf( fp_logRecg,"Module:[0103]\n" );
	}
	cv::Mat im_depth3;
	cv::Mat im_depth; 
	//RESIZE �摜�f�[�^�̃��T�C�Y
	cv::resize( d_img, im_depth, cv::Size(), SCALE, SCALE );

	// �J���[�摜�̓ǂݍ���
	cv::Mat im_color;
	//RESIZE �摜�f�[�^�̃��T�C�Y
	cv::resize( color_img, im_color, cv::Size(), SCALE, SCALE );


	//[0104] �����f�[�^�̓ǂݍ��� -> 2016.04.24�V���������d�l�ɕύX
	//[0104a] CNN�̌��ʂ̓ǂݍ���
	// hyp �Ɏp������������D
	if( log ){
		fprintf( fp_logRecg,"CNN�����f�[�^�ǂݍ���\n" );
		fprintf( fp_logRecg,"Module:[0104]\n" );
	}
	FILE		*fp_hyp;
	int			n_hyp;
	std::vector<Hyp> hyp;

	if( use_CNN_DATA ){
		if( (fp_hyp = fopen( hyp_fname, "r" )) == NULL ){
			fprintf( SE,"!!Error!! Module:[0102a]\n" );
			fprintf( SE,"  %s could not be read.\n", hyp_fname );
			return false;
		}else{
			Hyp tmp_hyp;
			int hyp_id, grasp_x, grasp_y, item_id, hoge;
			while( fscanf( fp_hyp,"%d,%d,%d,%d,%d", &hyp_id, &grasp_x, &grasp_y, &item_id, &hoge ) != EOF ){
				tmp_hyp.grasp.x = grasp_x;
				tmp_hyp.grasp.y = grasp_y;
				tmp_hyp.label = item_id;
				tmp_hyp.score = DL_SCORE; //DeepLearning�̐M���x�i����͌��߂����j
				tmp_hyp.method = CNN_LABEL;

				hyp.push_back( tmp_hyp );
			}
		
		}
		n_hyp = hyp.size();
		fclose( fp_hyp );
	}else{
		if( log ){
			fprintf( fp_logRecg,"CNN�̉����f�[�^��ǂݍ��݂܂���D\n" );
		}
	}


	//[0105] CascadedFAST�̌��ʂ̓ǂݍ���
	if( log ){
		fprintf( fp_logRecg,"FAST�����f�[�^�ǂݍ���\n" );
		fprintf( fp_logRecg,"Module:[0105]\n" );
	}
	if( use_FAST_DATA ){
		FILE		*fp_fast;
		std::vector<int> fast_i;
		std::vector<int> fast_j;
		std::vector<int> fast_id;
		std::vector<int> fast_n_match;
		std::vector<double> fast_score;
		if( (fp_fast = fopen( fast_hyp_fname, "r" )) == NULL ){
			if( log ){ 
				fprintf( fp_logRecg,"!!Error!! Module:[0105]\n" );
				fprintf( fp_logRecg,"  %s could not be read.\n", fast_hyp_fname );
				fclose( fp_logRecg );
			}
			return false;
		}else{
			Hyp tmp_hyp;
			int i, j, id, n_match;
			double score;
			while( fscanf( fp_fast,"%d,%d,%d,%d,%lf", &i, &j, &id, &n_match, &score ) != EOF ){
				fast_i.push_back( i );
				fast_j.push_back( j );
				fast_id.push_back( id );
				fast_n_match.push_back( n_match );
				fast_score.push_back( score );
			}
		
		}
		fprintf( SE,"%d fast keypoints are read.\n", fast_i.size() );
		if( log ) fprintf( fp_logRecg,"%d fast keypoints are read.\n", fast_i.size() );
		fclose( fp_fast );
		debug_FAST_res( &im_depth3, fast_i, fast_j, fast_id, fast_n_match );
		//Cascaded FAST�̌��ʂ������f�[�^�Ƃ��Ď�荞��
		for( int i=0 ; i<fast_id.size() ; i++ ){
			Hyp tmp_hyp;
			tmp_hyp.grasp.x = fast_i[i];
			tmp_hyp.grasp.y = fast_j[i];
			tmp_hyp.label = fast_id[i];
			tmp_hyp.score = fast_score[i];
			tmp_hyp.method = FAST_LABEL;

			hyp.push_back( tmp_hyp );
		}
		n_hyp = hyp.size();
	}else{
		if( log ){
			fprintf( fp_logRecg,"FAST�̉����f�[�^��ǂݍ��݂܂���D\n" );
		}
	}




	//--------------------------------------------------------------------------------//
	//[04 �Z�O�����e�[�V����]
	//--------------------------------------------------------------------------------//

	time1 = static_cast<double>(cv::getTickCount());
	double Seg_time;
	//[0402] �Z�O�����e�[�V����
	//[0402a] smoothing
	if( log ){
		fprintf( fp_logRecg,"�Z�O�����e�[�V����.\n");
		fprintf( fp_logRecg,"Module:[0402]\n" );
	}

	cv::Mat im_depth_smooth = im_depth.clone();
	for(int i = 0; i < 2; i++)
	{
	cv::medianBlur( im_depth_smooth, im_depth_smooth, 25 );
	}
	if( use_viewer ){
		cv::imshow( "im_depth_smooth", im_depth_smooth );
		cv::waitKey();
	}

	//--------------------------------------------------------------------------------//
	//�Z�O�����e�[�V�������@��3��ޑI�ׂ܂�
	//1.�f�v�X
	//2.�f�v�X�{�e�N�X�`��
	//3.�@������
	//�����ł�im_depth_edge����L��3��ނ̂ǂꂩ�̕��@�ō쐬���܂��D
	//--------------------------------------------------------------------------------//
	//�Z�O�����e�[�V�����̋��ʓI�ȕϐ�
	int m_nPrmCannyEdge = 30;
	cv::Mat im_depth_edge = cv::Mat::zeros(im_depth_smooth.rows, im_depth_smooth.cols, CV_8U);
	cv::Mat C =	cv::Mat::ones(3, 3, CV_32FC1);

	if( use_NORMAL_SEG ){
		//�@���������g�����Z�O�����e�[�V����
		cv::Mat im_NormalMap;
		int PCASize			= 11;	//	�@������Ɏg����`�T�C�Y(���f�B�A���t�B���^�̃T�C�Y���炢������ȏオ�����ł�)
		int ResizeRate		= 4;	//	�������ԂɊ�^���܂��D�摜�����T�C�Y�̊��� (Size�� 1/ResizeRate �ɂȂ�)
		unsigned char range_th	= 140;	// 20160623 �ђˁ@�ǉ��@�����摜�̃����W���L���邽�߂̃p�����[�^�i0-255�j0������255����O (�����l�F140)
		float noise_theta_th	= 65.0; // 20160623 �ђˁ@�ǉ��@���ʔ��˃m�C�Y���������߂̃p�����[�^�i0.0-90.0�j�������قǃm�C�Y��������i�����l�F65.0�j

		// top, front, side ��3�ʂɕ���
		// ���ӁF臒l���ׂ������̏�ԁ@���_�ɉ����ĕύX����K�v���� 20160511 
		cv::Mat top = cv::Mat::zeros(im_depth_smooth.rows, im_depth_smooth.cols, CV_8U);	// 20160621 domae �z��m�ۏC�� -���ɖ��Ȃ�
		cv::Mat front = cv::Mat::zeros(im_depth_smooth.rows, im_depth_smooth.cols, CV_8U);
		cv::Mat side = cv::Mat::zeros(im_depth_smooth.rows, im_depth_smooth.cols, CV_8U);

		calcNormalMap_recg( &im_depth_smooth, PCASize, ResizeRate, backgroundID, range_th, noise_theta_th, log_img, &top, &front, &side );	// 20160623 �ђ� range_th, noise_theta_th, log_img �̒ǉ��COutImg�̍폜
		//calcNormalMap_recg( &im_depth_smooth, PCASize, ResizeRate, backgroundID, &im_NormalMap, &top, &front, &side );	// 20160621 ���͉摜��raw�����摜�ɏC��

		cv::Mat t1, t2, t3;
		cv::Canny(top, t1, m_nPrmCannyEdge/2, m_nPrmCannyEdge, 3 );
		cv::Canny(front, t2, m_nPrmCannyEdge/2, m_nPrmCannyEdge, 3 );
		cv::Canny(side, t3, m_nPrmCannyEdge/2, m_nPrmCannyEdge, 3 );

		cv::bitwise_or( im_depth_edge, t1, im_depth_edge );
		cv::bitwise_or( im_depth_edge, t2, im_depth_edge );
		cv::bitwise_or( im_depth_edge, t3, im_depth_edge );

//		if(1){
//			cv::imshow( "top", t1 );
//			cv::imshow( "front", t2 );
//			cv::imshow( "side", t3 );
//			cv::waitKey(0);
//			cv::imshow( "edge", im_depth_edge );
//			cv::waitKey(0);
//
//		}

		cv::dilate( im_depth_edge, im_depth_edge, C);
	}else{

		//0402b - 0402c�܂łŃf�v�X�Z�O�����e�[�V����
		//
		//[0402b] edge detection
		cv::Canny( im_depth_smooth, im_depth_edge, 0, 15, 3 );
		//cv::imshow( "im_depth_edge", im_depth_edge );
		//cv::waitKey();

		//[0402c] dilation
		cv::dilate( im_depth_edge, im_depth_edge, C);
		//cv::imshow( "im_depth_edge", im_depth_edge );
		//cv::waitKey();


		//////////////////////////////////////////////////////////////
		// 2016.05.19
		// �e�N�X�`���Z�O�����e�[�V����
		// �f�v�X�G�b�W�Ƀe�N�X�`���G�b�W��ǉ����鏈���D
		//////////////////////////////////////////////////////////////
		if( use_TEX_SEG ){
			cv::Mat im_grey;
			cv::cvtColor( im_color, im_grey, CV_RGB2GRAY);
			cv::normalize( im_grey, im_grey, 0, 255, cv::NORM_MINMAX );
			cv::medianBlur( im_grey, im_grey, 5 );
			for(int i = 0; i < 2; i++)
			{
				cv::medianBlur( im_grey, im_grey, 15 );
			}
			//cv::imshow( "im_grey", im_grey );
			//cv::waitKey();
			cv::Mat im_grey_edge;
			cv::Canny( im_grey, im_grey_edge, 50, 80, 3 );
			if( log_img ){
				cv::imwrite( "im_gray.bmp", im_grey );
				cv::imwrite( "im_g_edge.bmp", im_grey_edge );
				cv::imwrite( "im_d_edge.bmp", im_depth_edge );
			}
			if( use_viewer ){
				cv::imshow( "im_grey_edge", im_grey_edge );
				cv::waitKey();
			}
			cv::dilate( im_grey_edge, im_grey_edge, C);
			//cv::imshow( "im_grey_edge", im_grey_edge );
			//cv::waitKey();

			if( use_viewer ){
				cv::imshow( "im_depth_edge", im_depth_edge );
				cv::waitKey();
			}
			//�����Ńf�v�X�G�b�W�Ƀe�N�X�`���G�b�W��ǉ�����
			for( int j=0; j<im_grey_edge.rows ; j++ ){
				for( int i=0; i<im_grey_edge.cols ; i++ ){
					if( im_grey_edge.at<uchar>( j, i ) == 255 ){
						im_depth_edge.at<uchar>( j, i ) = 255;
					}
				}
			}
			if( use_viewer ){
				cv::imshow( "im_depth_texture_edge", im_depth_edge );
				cv::waitKey();
			}
		}
		//////////////////////////////////////////////////////////////
	}



	cv::imwrite( "im_depth_edge.bmp", im_depth_edge );

	//[0402d] �G�b�W�}�X�N
	cv::Mat im_edge_mask;
	im_edge_mask	=	(im_depth_edge==0) - (im_depth_smooth==0);
	if( use_viewer ){
		cv::imshow( "im_edge_mask", im_edge_mask );
		cv::waitKey();
	}

	//[0402d_] opening 
	cv::Mat im_edge_mask2;
	cv::erode( im_edge_mask, im_edge_mask2, C );
	for( int i=0 ; i<n_closing_cluster-1 ; i++ ){
		cv::erode( im_edge_mask2, im_edge_mask2, C);
	}
	for( int i=0 ; i<n_closing_cluster ; i++ ){
		cv::dilate( im_edge_mask2, im_edge_mask2, C);
	}
	if( use_viewer ){
		cv::imshow( "im_edge_mask2", im_edge_mask2 );
		cv::waitKey();
	}

	//[0402e] ���x�����O
	LabelingBS	lab;
	int	n_segment; //�Z�O�����g��
	cv::Mat im_label( im_depth.rows, im_depth.cols, CV_16UC1 );
	lab.Exec(
		im_edge_mask.data,
		(short *)&im_label.data[0],
		im_depth.cols,
		im_depth.rows,
		true,
		min_segment_size
		);
	n_segment = lab.GetNumOfResultRegions();
	int lab_num2 = lab.GetNumOfResultRegions();



	// �Z�O�����g���̊m�F
	if( log ) fprintf( fp_logRecg,"n_segment_d: %d\n", n_segment );
	if( log ) fprintf( fp_logRecg,"n_segment_c: %d\n", lab_num2 );


	// �Z�O�����e�[�V�������ʂ̕\��
	cv::Mat im_label_nrm;
	cv::normalize( im_label, im_label_nrm, 0, 255, 32, CV_8UC1 );	//���K��
	if( use_viewer ){
		cv::imshow( "im_label", im_label_nrm );
		cv::waitKey();
	}


	//[0403A] 2016.06.20 �H���ǉ�
	//�e�Z�O�����g�̉摜�𐶐�����D
	std::vector<cv::Mat> im_seg;
	im_seg.resize( lab_num2+1 );
	for( int k=0 ; k<lab_num2+1 ; k++ ){
		im_seg[k] = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	}
	for( int j=0 ; j<HEIGHT ; j++ ){
		for( int i=0 ; i<WIDTH ; i++ ){
			im_seg[(int)im_label.at<short>( j, i )].at<uchar>( j, i ) = 255;
		}
	}
//	if(1){
//		char seg_name[256];
//		for( int k=0 ; k<lab_num2+1 ; k++ ){
//			sprintf( seg_name,"seg%d.bmp", k );
//			cv::imwrite( seg_name, im_seg[k] );
//		}
//	}

	time2 = static_cast<double>(cv::getTickCount());
	Seg_time = (time2 - time1)*1000 / cv::getTickFrequency();




	//-------------------------------------------------------------------//
	// �c���_�𑝂₷����
	//-------------------------------------------------------------------//
	// �Z�O�����g��0�Ԃ͔w�i�Ȃ̂Œ��ӂ��邱��
	std::vector<cv::Point> seg_center; //�Z�O�����g�d�S���W
	std::vector<int> seg_n_pix;        //�Z�O�����g�ʐ�
	std::vector<std::vector<cv::Point>> seg_pix; //�Z�O�����g���W
	cv::Point tmp_seg_pix;

	if( use_SegCenter ){
		seg_center.resize( lab_num2+1 );
		seg_n_pix.resize( lab_num2+1 );
		seg_pix.resize( lab_num2+1 );
		for( int k=0 ; k<lab_num2+1 ; k++ ){
			seg_center[k].x = 0;
			seg_center[k].y = 0;
			seg_n_pix[k] = 0;
			seg_pix[k].clear();
		}
		for( int j=0 ; j<HEIGHT ; j++ ){
			for( int i=0 ; i<WIDTH ; i++ ){
				seg_center[(int)im_label.at<short>( j, i )].x += i;
				seg_center[(int)im_label.at<short>( j, i )].y += j;
				seg_n_pix[(int)im_label.at<short>( j, i )]++;

				if( (int)im_label.at<short>( j, i ) != 0 ){ //�������ߖ�̂��߁C0�ԁi�w�i�j�̉�f�͊o���Ȃ��D
					tmp_seg_pix.x = i;
					tmp_seg_pix.y = j;
					seg_pix[(int)im_label.at<short>( j, i )].push_back( tmp_seg_pix );
				}
			}
		}
		for( int k=0 ; k<lab_num2+1 ; k++ ){
			seg_center[k].x = (int)( (double)seg_center[k].x / (double)seg_n_pix[k] );
			seg_center[k].y = (int)( (double)seg_center[k].y / (double)seg_n_pix[k] );
		}
	}



	//[0200] Cascaded FAST �ɂ��c���_�F��
	time1 = static_cast<double>(cv::getTickCount());
	double FAST_time;
	if( use_FAST ){
		if( log ) fprintf( fp_logRecg,"Fast_____Start.\n");
		std::vector<Hyp> fast_hyp;
		RunCascadedFast( im_color, im_depth, viewpoint, nItemIndices, &nItemNum, fast_hyp ) ;
		for( int i=0 ; i<fast_hyp.size() ; i++ ){
			hyp.push_back( fast_hyp[i] );
		}
		n_hyp = hyp.size();
		if( log ) fprintf( fp_logRecg,"Fast_____End.\n");
		// FAST�̔c���_�̔F�����ʂ̕ۑ�
		if( log_img2 ){
			saveHypotheses( &im_color, fast_hyp, "result_FAST.bmp" );
		}

		//���J�� �ȉ���2�s��ǉ� --2016.06.01
		fast_hyp.clear();
		std::vector<Hyp> ().swap(fast_hyp);
		
	}

	time2 = static_cast<double>(cv::getTickCount());
	FAST_time = (time2 - time1)*1000 / cv::getTickFrequency();
	fprintf( SE,"Fast end\n");

	//[0300] �c���_�F��
	time1 = static_cast<double>(cv::getTickCount());
	double FGE_time;
	fprintf( SE,"FGE start\n" );
	std::vector<cv::Point> grasp_v;
	if( !use_SegCenter ){
		//run_FGE() �̑�3������ 
		//2:�z���C3:����
		if( run_FGE( im_depth, im_color, segMethod, viewpoint, 2, grasp_v, log ) ){
		}
		fprintf( SE,"FGE end\n" );
		for( int i=0 ; i<grasp_v.size() ; i++ ){
			fprintf( SE,"grasp_v( %d, %d )\n", grasp_v[i].x, grasp_v[i].y );
		}
	}
	time2 = static_cast<double>(cv::getTickCount());
	FGE_time = (time2 - time1)*1000 / cv::getTickFrequency();


	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////
	// �c���_���폜���āC�Z�O�����g�d�S�ɓ���ւ�
	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////
	std::vector<bool> flag_seg_centroid;
	if( use_SegCenter ){
		grasp_v.clear();

		// �Z�O�����g�d�S�����ʓ_�ɂ�����@������
		//for( int k=1 ; k<lab_num2+1 ; k++ ){
		//	grasp_v.push_back( seg_center[k] );
		//}
		// �Z�O�����g�d�S�����ʓ_�ɂ�����@������


		//2016.06.23 �Z�O�����g��Ɋm���Ɏ��ʓ_�𐶐�������@
		if( log ) fprintf( fp_logRecg,"\n\nGenerate grasp point\n" );
		int grasp_id, n_register;
		for( int k=1 ; k<lab_num2+1 ; k++ ){
			//�܂��̓Z�O�����g�̐^�񒆂Ɉ�_�o�^
			//��{�̓Z�O�����g�d�S�����ǁC
			if( (int)im_label.at<short>( seg_center[k].y, seg_center[k].x ) == k ){
				if( log ) fprintf( fp_logRecg,"Seg[%d] -> [centroid]\n", k );
				grasp_v.push_back( seg_center[k] );
				flag_seg_centroid.push_back( true );
			}else{ //�d�S�������������fid�̐^�񒆂ɂ���D
				if( log ) fprintf( fp_logRecg,"Seg[%d] %d th pix [center id]\n", k, seg_n_pix[k]/2 );
				grasp_v.push_back( seg_pix[k][seg_n_pix[k]/2] );
				flag_seg_centroid.push_back( true );
			}

			n_register = (int)((double)seg_n_pix[k] / (double)grasp_step);
			for( int i=0 ; i<n_register ; i++ ){
				grasp_id = (int)( ((double)rand()/(double)RAND_MAX) * (double)(seg_n_pix[k]-1) );
				grasp_v.push_back( seg_pix[k][grasp_id] );
				flag_seg_centroid.push_back( false );
					
				if( log ) fprintf( fp_logRecg,"Seg[%d] %d th pix total:%d\n", k, grasp_id, seg_n_pix[k] );
			}

			// �\�萔�̎��ʓ_���������ꂽ�ꍇ�͏I���D
			if( max_grasp_point < grasp_v.size() ) break;
		}

		//2016.06.25 �c���_�����Ȃ�����ꍇ�͔c���_��ǉ��D
		if( grasp_v.size() < min_grasp_point ){
			int loop_cnt; //�������[�v�Ɋׂ�Ȃ��悤�ɂ��邽�߂̕ϐ�
			loop_cnt=0;
			while( grasp_v.size() < min_grasp_point ){
				for( int k=1 ; k<lab_num2+1 ; k++ ){
		
					grasp_id = (int)( ((double)rand()/(double)RAND_MAX) * (double)(seg_n_pix[k]-1) );
					grasp_v.push_back( seg_pix[k][grasp_id] );
					flag_seg_centroid.push_back( false );
						
					if( log ) fprintf( fp_logRecg,"Seg[%d] %d th pix total:%d\n", k, grasp_id, seg_n_pix[k] );
				}
				loop_cnt++;
				// ���[�v��10������̂͂��������̂�break����D
				if( 10 < loop_cnt ){
					if( log ) fprintf( fp_logRecg,"�c���_�𑝂₷�u���b�N�Ŗ������[�v�������D\n" );
					break;
				}
			}
		}
		
	}


	//�c���_����������ꍇ�͍폜����D
	//���炩�̃A���S���Y��������Ƃ����Ǝv�����C����͖������폜
	if( max_grasp_point < grasp_v.size() ){
		int n_reduce;
		n_reduce = grasp_v.size() - max_grasp_point;
		fprintf( SE,"all grasp point:%d\n", grasp_v.size() );
		fprintf( SE,"n_reduce:%d\n", n_reduce );
		for( int i=0 ; i<n_reduce ; i++ ){
			grasp_v.pop_back();
			flag_seg_centroid.pop_back();
		}
	}
	fprintf( SE,"Reduced grasp point:%d\n", grasp_v.size() );
	for( int i=0 ; i<grasp_v.size() ; i++ ){
		fprintf( SE,"grasp_v( %d, %d )\n", grasp_v[i].x, grasp_v[i].y );
	}



	///* �Z�O�����g�Ǝ��ʓ_�̉���
	if( log_img ){
		char seg_ofname[2048];
		cv::Mat label(im_label.size(), CV_8UC3, cv::Scalar(255, 255, 255));
		for (int i = 0; i < n_segment; i++)
		{
			// ���x�����O���ʂ̗̈�𒊏o����B
			cv::Mat labelarea;
			compare(im_label, i+1, labelarea, CV_CMP_EQ);
			//
			// ���o�����̈�Ƀ����_���F��ݒ肷��B
			cv::Scalar randomcolor = cv::Scalar(rand()&0xFF, rand()&0xFF, rand()&0xFF);
			cv::Mat color(im_label.size(), CV_8UC3, randomcolor);
			color.copyTo(label, labelarea);
		}
		for( int i=0 ; i<grasp_v.size() ; i++ ){
			if( flag_seg_centroid[i] ){
				cv::circle( label, grasp_v[i], 8, cv::Scalar( 0, 0, 255 ), -1, CV_AA ); //�F�̎w���BGR�̏�
				cv::circle( label, grasp_v[i], 5, cv::Scalar( 255, 255, 255 ), -1, CV_AA ); //�F�̎w���BGR�̏�
			}else{
				cv::circle( label, grasp_v[i], 8, cv::Scalar( 255, 0, 0 ), -1, CV_AA ); //�F�̎w���BGR�̏�
				cv::circle( label, grasp_v[i], 5, cv::Scalar( 255, 255, 255 ), -1, CV_AA ); //�F�̎w���BGR�̏�
			}
		}
		sprintf( seg_ofname, "segment_grasp_point.bmp" );
		imwrite( seg_ofname, label );
	}
	

	//[0300] CNN �ɂ��c���_�F��
	time1 = static_cast<double>(cv::getTickCount());
	double CNN_time;
	if( use_CNN ){
		if( log ) fprintf( fp_logRecg,"CNN_____Start.\n");
		std::vector<Hyp> cnn_hyp;
		fprintf( SE,"CNN_____Start.\n");
		if( RunCNN( im_color, im_depth, nItemIndices, &nItemNum, nTargetItemIdx, grasp_v, cnn_hyp, viewpoint ) ){
			fprintf( SE,"CNN_END.\n");
			for( int i=0 ; i<cnn_hyp.size() ; i++ ){
				hyp.push_back( cnn_hyp[i] );
			}
			n_hyp = hyp.size();
			if( log ) fprintf( fp_logRecg,"CNN_____End.\n");
		}else{
			if( log ) fprintf( fp_logRecg,"!!Error. RunCNN()\n");
		}
		// CNN�̔c���_�̔F�����ʂ̕ۑ�
		if( log_img2 ){
			saveHypotheses( &im_color, cnn_hyp, "result_CNN.bmp" );
		}

		//���J�� �ȉ���2�s��ǉ� --2016.06.01
		cnn_hyp.clear();
		std::vector<Hyp> ().swap(cnn_hyp);

	}

	time2 = static_cast<double>(cv::getTickCount());
	CNN_time = (time2 - time1)*1000 / cv::getTickFrequency();

	fprintf( SE,"CNN end\n");


	//���J�� �ȉ���4�s��ǉ� --2016.06.01
	grasp_v.clear();
	std::vector<cv::Point> ().swap(grasp_v);

	//�c���ʒu�̃X�P�[�����O�i���͉摜�̏k���䗦�ɍ��킹�Ƃ��j
	for( int i=0 ; i<n_hyp ; i++ ){
		hyp[i].grasp.x = (int)(SCALE*(double)hyp[i].grasp.x);
		hyp[i].grasp.y = (int)(SCALE*(double)hyp[i].grasp.y);
	}




	//[01G] �l�H�I�Ȉ�`�q�̓ǂݍ���
	if( log ) fprintf( fp_logRecg,"�l�H�I�Ȉ�`�q�̓ǂݍ���.\n");
	std::vector<individuall> art_gene;
	if( use_artifitial ){
		if( log ) fprintf( fp_logRecg,"Additional module: use artifitial gene.\n");
		FILE	*fp_art;
		int		n_art, bit;
		if( (fp_art = fopen( artfname, "r" )) == NULL ){
			if( log ){
				fprintf( fp_logRecg,"Artifitial gene is not read.\n");
				fclose( fp_logRecg );
			}
			return false;
		}
		fscanf( fp_art,"%d", &n_art );
		art_gene.resize( n_art );
		for( int j=0 ; j<n_art ; j++ ){
			art_gene[j].chrom.resize( n_hyp );
			art_gene[j].born = MODIFIED;
			for( int i=0 ; i<n_hyp ; i++ ){
				fscanf( fp_art,"%d", &bit );
				if( bit == 1 ) art_gene[j].chrom[i] = true;
				else		   art_gene[j].chrom[i] = false;

			}

		}
	}

	//double time_load_data;
	//time_load_data = timer->getTime() - processing_time;
	//processing_time = timer->getTime() ;

	//[0403] �����f�[�^�ɋA������Z�O�����gid�����蓖�Ă�
	time1 = static_cast<double>(cv::getTickCount());
	double Seg_assign_time;
	for( int i=0 ; i<n_hyp ; i++ ){
		hyp[i].segIdx = (int)im_label.at<unsigned short>( hyp[i].grasp.y, hyp[i].grasp.x );
		if( log ) fprintf( fp_logRecg,"hyp:%d -> seg:%d\n", i, hyp[i].segIdx );
		if( log ) fprintf( fp_logRecg,"          grasp( %d, %d )\n", hyp[i].grasp.x, hyp[i].grasp.y );
		if( hyp[i].segIdx == 0 ){
			if( log ) fprintf( fp_logRecg,"need update\n" );
			int new_label;
			cv::Point new_grasp;
			searchNeasestSegment( hyp[i].grasp.x, hyp[i].grasp.y, &im_label, &new_label, &new_grasp );
			hyp[i].segIdx = new_label;
			hyp[i].grasp = new_grasp;
			if( log ) fprintf( fp_logRecg,"    hyp:%d -> seg:%d\n\n", i, hyp[i].segIdx );
			if( log ) fprintf( fp_logRecg,"              grasp( %d, %d )\n", hyp[i].grasp.x, hyp[i].grasp.y );
		}
		//cv::imshow( "segment", hyp[i].segment );
		if( log ) fprintf( fp_logRecg,"grasp(%d,%d)\n", hyp[i].grasp.x, hyp[i].grasp.y );
	}
	//fprintf( SE,"Number of segment: %d\n", n_segment );
	//cv::waitKey();


	//[0404] �Z�O�����g�����肠����Ȃ������������폜
	//       �L�[�|�C���g�Ɖ�������������
	std::vector<Hyp> hyp2;
	for( int i=0 ; i<n_hyp ; i++ ){
		//if( hyp[i].segIdx != 0 ){
		if( (hyp[i].segIdx != 0) && (hyp[i].label != 0) ){ //2016.06.24 �H�� �w�i���x�����������ʓ_���폜
			hyp2.push_back( hyp[i] );
		}
	}
	hyp.clear();
	for( int i=0 ; i<hyp2.size() ; i++ ){
		hyp.push_back( hyp2[i] );
	}
	n_hyp = hyp.size();
	hyp2.clear();


	time2 = static_cast<double>(cv::getTickCount());
	Seg_assign_time = (time2 - time1)*1000 / cv::getTickFrequency();

	fprintf( SE,"Seg end\n");

	//�����f�[�^���Ȃ��ꍇ��false
	if( hyp.size() == 0 ){

//#ifdef DEBUG_02	// ������ 20160621 ���ؗp 
FILE	*fp_deb1=0;
fopen_s(&fp_deb1, "0001_RecgAPC2016_DEBUG_LOG.txt", "w");
fprintf_s(fp_deb1, "�����f�[�^���Ȃ��̂�false\n");
fclose(fp_deb1);
//#endif

		fclose( fp_logRecg );
		return false;
	}

	
	//FILE *fp_cnn;
	//if( log ){
	//	fp_cnn = fopen( "cnn.txt", "w" );
	//}
	//2016.06.25 CNN�̃X�R�A�̕␳
	int patch_size;
	patch_size = 165;
	for( int i=0 ; i<n_hyp ; i++ ){
		//if( hyp[i].method == CNN_LABEL ){
		if( (hyp[i].method == CNN_LABEL) 
			&& (hyp[i].label!=36) && (hyp[i].label!=39) ){ //���u���V�͂��Ƃ��ƃp�b�`���f�[�^�����Ȃ��̂œK�p�O�D

			cv::Mat patch;
			int area;
			area = 0;
			//cv::getRectSubPix(im_seg[hyp[i].segIdx], cv::Size(patch_size, patch_size), hyp[i].grasp, patch);
			//�Z�O�����g���ƍׂ�������ꍇ������̂ŁC�����摜�ɕύX
			cv::getRectSubPix( im_depth_smooth, cv::Size(patch_size, patch_size), hyp[i].grasp, patch );
			area = cv::countNonZero( patch );
			//fprintf( fp_cnn,"hyp[%d] grasp(%d, %d) score: %lf \n", i, hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].score );

			//hyp[i].score = hyp[i].score * tanh( 3.5*(double)((double)area/(double)(patch_size*patch_size)) );
			hyp[i].score = hyp[i].score * pow( (double)((double)area/(double)(patch_size*patch_size)), 1.0/8.0 );
			//fprintf( fp_cnn,"  area: %lf\n", (double)area / (patch_size*patch_size) );
			//fprintf( fp_cnn,"  -> new score: %lf\n", hyp[i].score );
		}
	}
	//fclose( fp_cnn );

	//--------------------------------------------------------------------------------//
	//[05 Genetic Algorithm]
	//--------------------------------------------------------------------------------//
	fprintf( SE,"GA start\n");
	time1 = static_cast<double>(cv::getTickCount());
	double GA_time;

	//[0500] ���O�p�̕ϐ��̗p�ӁiGA�̏����Ɗ֌W�Ȃ��̂ŁC�������Ԃ̌v�Z�ΏۊO�Ƃ����D�j
	int	log_n_generate; //�������ꂽ���̐�

	log_n_generate = population;
	//[0501] ��`�I�A���S���Y���ɂ��F���J�n
	if( log ){
		fprintf( fp_logRecg,"Start GA\n" );
		fprintf( fp_logRecg,"Module:[0500]\n" );
	}

	FILE *fp_log;
	if( log ){
		fp_log = fopen( "log.txt", "w" );
	}

	GA	su; //scene understanding
	std::vector<double>	max_fitness;
	std::vector<double>	ave_fitness;
	su.generateInitialIndividuals( population, hyp.size(), n_on ); 

	// [0502] �l�H�I�ɍ�����̂̑��
	if( use_artifitial ){
		for( int i=0 ; i<art_gene.size() ; i++ ){
			su.ind.pop_back();
		}
		for( int i=0 ; i<art_gene.size() ; i++ ){
			su.ind.push_back( art_gene[i] );
		}
		su.showIndividual( population-1 );
		getchar();
	}else{
		if( log ) fprintf( fp_logRecg,"Don't use artifitial gene.\n");
	}


	for( int i=0 ; i<su.getN_ind() ; i++ ){
		calcFitness( &im_label_nrm, hyp, n_item, n_segment, seg_n_pix, &su.ind[i] );
#if LOG_MODE
		su.showIndividual( i );
		//showSceneHypothesis( &im_depth3, hyp, &su.ind[i] );
#endif
	}

	if( use_artifitial ){
		su.showIndividual( su.getN_ind()-1 );
		showSceneHypothesis( &im_depth3, hyp, &su.ind[su.getN_ind()-1], "art_gene.bmp", n_segment, im_seg, im_label );

	}

	su.calcAveFitness();
	su.SortIndividuals();
	max_fitness.push_back( su.ind[0].fitness );
	ave_fitness.push_back( su.getAveFitness() );


	// Start genetic operation.
	for( int i=0 ; i<n_gen ; i++ ){

		if( 20 < rand()%100 ){
			su.crossover();
		}

		if( 97 < rand()%100 ){
			su.mutation();
		}

		// �q�̂̓K���x�v�Z
//		now = GAtimer->getTime();
		for( int k=0 ; k<su.child.size() ; k++ ){
			calcFitness( &im_label_nrm, hyp, n_item, n_segment, seg_n_pix, &su.child[k] );
		}

//		log_n_generate += su.child.size();
		//su.showIndividual( 0 );

		// �q�̂�e�̌̏W�c�֓o�^
		su.update();
		su.calcAveFitness();

		max_fitness.push_back( su.ind[0].fitness );
		ave_fitness.push_back( su.getAveFitness() );
		//============================
		// �I������
		//============================
		// n_stable_gen����ȏ�ω�������������I������D
		if( n_stable_gen < i ){
			if( max_fitness[i] == max_fitness[i-n_stable_gen] ){
				break;
			}
		}
	}
	if( log ) fclose( fp_log );

	time2 = static_cast<double>(cv::getTickCount());
	GA_time = (time2 - time1)*1000 / cv::getTickFrequency();
	
	//2016.06.25 �o�b�N�A�b�v
	individuall ind_cpy;
	ind_cpy.chrom.resize( su.ind[0].chrom.size() );
	for( int i=0 ; i<ind_cpy.chrom.size() ; i++ ){
		ind_cpy.chrom[i] = su.ind[0].chrom[i];
	}
	ind_cpy.fitness = su.ind[0].fitness;

	DeleteMultiLabel( hyp, n_segment, &su.ind[0] );

	fprintf( SE,"GA end\n");
	//--------------------------------------------------------------------------------//
	//[06 �ԋp�p�̔c���_�f�[�^���o��]
	//--------------------------------------------------------------------------------//
	time1 = static_cast<double>(cv::getTickCount());
	double Out_time;
	//2016.05.19
	//GA�̌��ʂ���C�^�[�Q�b�g�̃��x�������𔲂��o��
	std::vector<int> res_i;
	std::vector<int> res_j;
	std::vector<double> res_score;
	std::vector<int> resIdx;
	std::vector<int> hypIdx;
	int	cnt;
	cnt = 0;

	//�ђˁ@�ǉ��@-start
	std::vector<int> nt_res_i;
	std::vector<int> nt_res_j;
	std::vector<double> nt_res_score;
	std::vector<int> nt_resIdx;
	std::vector<int> nt_hypIdx;
	int	nt_cnt;
	nt_cnt = 0;
	//�ђˁ@�ǉ��@-end

	if( log ){
		FILE *fp_hyp;
		fp_hyp = fopen( "all_hyp.txt", "w" );
		fprintf( fp_hyp, " Id, Label\n" );
		for( int i=0 ; i<hyp.size() ; i++ ){
			fprintf( fp_hyp, "%03d, %03d\n", i, hyp[i].label );
		}
		fclose( fp_hyp );
	}

	cv::Mat im_cluster; //�N���X�^�摜
	cv::Mat im_nt_cluster; //�^�[�Q�b�g�ȊO�N���X�^
	cv::Mat im_objIdx;	   //id map
	cv::Mat im_id_mask;
	cv::Mat im_c_map;

	im_cluster = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	im_nt_cluster = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
	im_objIdx = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
	im_id_mask = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
	im_c_map = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

	cv::Mat CC =	cv::Mat::ones(9, 9, CV_32FC1); //�c���p

	if( nTargetItemIdx != 0 ){ //�^�[�Q�b�g��0�ȊO�̎��i�܂�C�r�����̕��̔F���j


		for( int i=0 ; i<su.ind[0].chrom.size() ; i++ ){
			if( (su.ind[0].chrom[i]) ){
				if( (hyp[i].label == nTargetItemIdx) ){
					if( log ){
						fprintf( fp_logRecg,"%4d,%4d,%d,%d,%.3lf", hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].label, hyp[i].segIdx, hyp[i].score);
						if( hyp[i].method == CNN_LABEL )       fprintf( fp_logRecg,",CNN\n");
						else if( hyp[i].method == FAST_LABEL ) fprintf( fp_logRecg,",Fast\n");
					}
					res_i.push_back( hyp[i].grasp.x );
					res_j.push_back( hyp[i].grasp.y );
					res_score.push_back( hyp[i].score );
					resIdx.push_back( cnt );
					hypIdx.push_back( i );
					cnt++;
				}else if( hyp[i].label != 0 ){ //2016.06.24 �H�� �����ǉ� nt �ɂ��w�i��n���Ȃ�
					if( log ){
						fprintf( fp_logRecg,"[nt]%4d,%4d,%d,%d,%.3lf", hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].label, hyp[i].segIdx, hyp[i].score);
						if( hyp[i].method == CNN_LABEL )       fprintf( fp_logRecg,",[nt]CNN\n");
						else if( hyp[i].method == FAST_LABEL ) fprintf( fp_logRecg,",[nt]Fast\n");
					}
					nt_itemIdx.push_back( hyp[i].label );
					nt_res_i.push_back( hyp[i].grasp.x );
					nt_res_j.push_back( hyp[i].grasp.y );
					nt_res_score.push_back( hyp[i].score );
					nt_resIdx.push_back( nt_cnt );
					nt_hypIdx.push_back( i );
					nt_cnt++;
				}
			}
		}


		if( CombSort( res_score, resIdx ) ){
			fprintf( SE,        "Sorted result\n");
			if( log ) fprintf( fp_logRecg,"Sorted result\n");
			for( int i=0 ; i<res_score.size() ; i++ ){
				fprintf( SE,        "id: %d, %4d, %4d, %.3lf\n", resIdx[resIdx[i]], res_i[resIdx[i]], res_j[resIdx[i]], res_score[resIdx[i]] );
				if( log ) fprintf( fp_logRecg,"id %d: %d, %4d, %4d, %.3lf\n", resIdx[resIdx[i]], res_i[resIdx[i]], res_j[resIdx[i]], res_score[resIdx[i]] );
			}
		}else{
			if( log ){
				fprintf( fp_logRecg,"�\�[�g�̃G���[\n");
				fprintf( fp_logRecg,"�c���_��M���x�ɂ���ĕ��ёւ����ɏo�͂��܂��D\n");
				for( int i=0 ; i<res_score.size() ; i++ ){
					fprintf( fp_logRecg,"id: %d %d, %4d, %4d, %.3lf\n", resIdx[resIdx[i]], res_i[resIdx[i]], res_j[resIdx[i]], res_score[resIdx[i]] );
				}
			}
		}
		//�ђˁ@�ǉ� -start

		if( CombSort( nt_res_score, nt_resIdx ) ){
			fprintf( SE,        "[nt]Sorted result\n");
			if( log ) fprintf( fp_logRecg,"[nt]Sorted result\n");
			for( int i=0 ; i<nt_res_score.size() ; i++ ){
				fprintf( SE,        "[nt]%4d, %4d, %.3lf\n", nt_res_i[nt_resIdx[i]], nt_res_j[nt_resIdx[i]], nt_res_score[nt_resIdx[i]] );
				if( log ) fprintf( fp_logRecg,"[nt]%4d, %4d, %.3lf, label: %d\n", nt_res_i[nt_resIdx[i]], nt_res_j[nt_resIdx[i]], nt_res_score[nt_resIdx[i]], hyp[ nt_hypIdx[nt_resIdx[i]] ].label );
			}
		}else{
			if( log ){
				fprintf( fp_logRecg,"[nt]�\�[�g�̃G���[\n");
				fprintf( fp_logRecg,"[nt]�c���_��M���x�ɂ���ĕ��ёւ����ɏo�͂��܂��D\n");
				for( int i=0 ; i<nt_res_score.size() ; i++ ){
					fprintf( fp_logRecg,"[nt]%4d, %4d, %.3lf\n, label: %d", nt_res_i[nt_resIdx[i]], nt_res_j[nt_resIdx[i]], nt_res_score[nt_resIdx[i]], hyp[ nt_hypIdx[nt_resIdx[i]] ].label );
				}
			}
		}
		//�ђˁ@�ǉ� -end





		//�o�̓t�@�C���ւ̃f�[�^�R�s�[
		for( int i=0 ; i<res_score.size() ; i++ ){
			work_i.push_back( res_i[resIdx[i]] );
			work_j.push_back( res_j[resIdx[i]] );
			work_score.push_back( res_score[resIdx[i]] );
		}
		//�ђˁ@�ǉ� -start
		//�o�̓t�@�C���ւ̃f�[�^�R�s�[
		for( int i=0 ; i<nt_res_score.size() ; i++ ){
			nt_i.push_back( nt_res_i[nt_resIdx[i]] );
			nt_j.push_back( nt_res_j[nt_resIdx[i]] );
			nt_score.push_back( nt_res_score[nt_resIdx[i]] );
		}
		//�ђˁ@�ǉ� -end



		//�Z�O�����g�摜�icluster�̍쐬�j
		for( int j=0 ; j<res_score.size() ; j++ ){
			cv::bitwise_or( im_cluster, im_seg[ hyp[hypIdx[j]].segIdx ], im_cluster );
		}
		//�^�[�Q�b�g�ȊO�̃Z�O�����g�摜�int_cluster�̍쐬�j
		for( int j=0 ; j<nt_res_score.size() ; j++ ){
			cv::bitwise_or( im_nt_cluster, im_seg[ hyp[nt_hypIdx[j]].segIdx ], im_nt_cluster );
		}


		//����ID����f�l�Ƃ����摜���쐬����(id_map�̍쐬)
		for( int j=0 ; j<res_score.size() ; j++ ){
			cv::bitwise_and( im_seg[hyp[hypIdx[j]].segIdx], hyp[hypIdx[j]].label, im_id_mask );
			cv::bitwise_or( im_objIdx, im_id_mask, im_objIdx );
		}
		for( int j=0 ; j<nt_res_score.size() ; j++ ){
			cv::bitwise_and( im_seg[hyp[nt_hypIdx[j]].segIdx], hyp[nt_hypIdx[j]].label, im_id_mask );
			cv::bitwise_or( im_objIdx, im_id_mask, im_objIdx );
		}

		//�M���x�}�b�v�ic_map�̍쐬�j
		std::vector<std::vector<int>> segIdxList; //�c���Z�O�����g���C�������Z�O�����g��Id�ɋA�����鉼����Id
		std::vector<double> segMeanScore; //�Z�O�����g���Ƃ̕��σX�R�A -> c_map�̒l�ɂȂ�
		segIdxList.resize( n_segment+1 );
		segMeanScore.resize( n_segment+1 );
		for( int j=0 ; j<segMeanScore.size() ; j++ ){
			segMeanScore[j] = 0.0;
		}
		for( int j=0 ; j<res_score.size() ; j++ ){ //segIdxList�̍쐬
			segIdxList[ hyp[hypIdx[j]].segIdx ].push_back( hypIdx[j] );
			segMeanScore[ hyp[hypIdx[j]].segIdx ] += hyp[hypIdx[j]].score;
		}
		for( int j=0 ; j<nt_res_score.size() ; j++ ){ //segIdxList�̍쐬
			segIdxList[ hyp[nt_hypIdx[j]].segIdx ].push_back( nt_hypIdx[j] );
			segMeanScore[ hyp[nt_hypIdx[j]].segIdx ] += hyp[nt_hypIdx[j]].score;
		}
		for( int j=0 ; j<segMeanScore.size() ; j++ ){
			if( 1 < segIdxList[j].size() ){
				segMeanScore[j] /= (double)segIdxList[j].size();
			}
		}

		for( int j=0 ; j<im_label.rows ; j++ ){
			for( int i=0 ; i<im_label.cols ; i++ ){
				im_c_map.at<uchar>( j, i ) = (unsigned char)( 100.0 * (double)segMeanScore[ im_label.at<short>( j, i ) ] ) ;
			}
		}

		cv::dilate( im_objIdx, im_objIdx, CC, cv::Point(-1,-1), 5 );
		cv::dilate( im_c_map, im_c_map, CC, cv::Point(-1,-1), 5 );

		//�^�[�Q�b�g�̗̈�̂�+100
		for( int j=0 ; j<im_label.rows ; j++ ){
			for( int i=0 ; i<im_label.cols ; i++ ){
				if( im_objIdx.at<uchar>( j, i ) == nTargetItemIdx ){ //�^�[�Q�b�g��ID�ɑ΂��Ắ{100����D
					im_c_map.at<uchar>( j, i ) += 100;
				}
			}
		}

	}else{ //�^�[�Q�b�g��0�̎��i�܂�C���u����ł̕��̔F���j

		std::vector<int> res_label;
		for( int i=0 ; i<su.ind[0].chrom.size() ; i++ ){
			if( (su.ind[0].chrom[i]) ){
				if( (hyp[i].label != 0) ){
					if( log ){
						fprintf( fp_logRecg,"%4d,%4d,%d,%d,%.3lf", hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].label, hyp[i].segIdx, hyp[i].score);
						if( hyp[i].method == CNN_LABEL )       fprintf( fp_logRecg,",CNN\n");
						else if( hyp[i].method == FAST_LABEL ) fprintf( fp_logRecg,",Fast\n");
					}
					res_i.push_back( hyp[i].grasp.x );
					res_j.push_back( hyp[i].grasp.y );
					res_score.push_back( hyp[i].score );
					res_label.push_back( hyp[i].label );
					resIdx.push_back( cnt );
					hypIdx.push_back( i );
					cnt++;
				}
			}
		}

		if( CombSort( res_score, resIdx ) ){
			fprintf( SE,        "Sorted result\n");
			if( log ) fprintf( fp_logRecg,"Sorted result\n");
			for( int i=0 ; i<res_score.size() ; i++ ){
				fprintf( SE,                  "id: %d, %4d, %4d, %.3lf, label = %d\n", resIdx[resIdx[i]], res_i[resIdx[i]], res_j[resIdx[i]], res_score[resIdx[i]], res_label[resIdx[i]] );
				if( log ) fprintf( fp_logRecg,"id: %d, %4d, %4d, %.3lf, label = %d\n", resIdx[resIdx[i]], res_i[resIdx[i]], res_j[resIdx[i]], res_score[resIdx[i]], res_label[resIdx[i]] );
			}
		}

		//�Z�O�����g�摜�icluster�̍쐬�j
		for( int j=0 ; j<res_score.size() ; j++ ){
			cv::bitwise_or( im_cluster, im_seg[hyp[hypIdx[j]].segIdx], im_cluster );
		}
		//id_map�̍쐬�i�ő�X�R�A�������̂�ID�Ƃ���j
		im_objIdx = cv::Mat::ones( im_objIdx.rows, im_objIdx.cols, CV_8UC1 ) * res_label[resIdx[0]];
		cv::bitwise_and( im_objIdx, im_cluster, im_objIdx );
		//�M���x�}�b�v�ic_map�̍쐬�j �P��̕��̂����Ȃ̂ŁC�ő�X�R�A�����Ă���
		im_c_map = cv::Mat::ones( im_c_map.rows, im_c_map.cols, CV_8UC1 ) * 200;
		cv::bitwise_and( im_c_map, im_cluster, im_c_map );


		cv::dilate( im_objIdx, im_objIdx, CC, cv::Point(-1,-1), 5 );
		cv::dilate( im_c_map, im_c_map, CC, cv::Point(-1,-1), 5 );

	}



	// �o�̓f�[�^���쐬�D�iuchar��1�����z��ɃR�s�[�j
	for( int i=0 ; i<im_c_map.cols * im_c_map.rows ; i++ ){
		cluster[i] = im_cluster.data[i];
		nt_cluster[i] = im_nt_cluster.data[i];
		id_map[i] = im_objIdx.data[i];
		c_map[i] = im_c_map.data[i];
	}




	time2 = static_cast<double>(cv::getTickCount());
	Out_time = (time2 - time1)*1000 / cv::getTickFrequency();
	//--------------------------------------------------------------------------------//
	//[07 Post processing]
	//--------------------------------------------------------------------------------//
	time1 = static_cast<double>(cv::getTickCount());
	double Post_time;


	// ���O�t�@�C���̏����o��
	if( log ){
		if( log_img ){
			showSceneHypothesis( &im_depth, hyp, &su.ind[0], "result.bmp", n_segment, im_seg, im_label );
			//debug_score_MA( &im_label, hyp, &su.ind[0] );
			// �e�L�g�[�Ȍ̂̃r�b�g��S���P�ɂ��āC����
			saveAllHypotheses( &im_depth, hyp, &su.ind[0], "result_all.bmp", n_segment, im_seg, im_label );

			showSceneHypothesis( &im_depth, hyp, &ind_cpy, "result_before_delete.bmp", n_segment, im_seg, im_label );
		}
		//�ŏI�I�ȉ����e�L�X�g�\��
		FILE	*fp_result;
		fp_result = fopen( "result_chrom.txt", "w" );
		for( int i=0 ; i<su.ind[0].chrom.size() ; i++ ){
			if( su.ind[0].chrom[i] ){
				fprintf( fp_result,"1\n");
			}else{
				fprintf( fp_result,"0\n");
			}
		}
		fclose( fp_result );
	

	}

	if( log_img2 ){
		saveSegmentation( im_seg, "result_seg.bmp" );

		cv::imwrite( "output_c_map.bmp", im_c_map );
		cv::imwrite( "output_im_cluster.bmp", im_cluster );
		cv::imwrite( "output_im_nt_cluster.bmp", im_nt_cluster );
		cv::imwrite( "output_id_map.bmp", im_objIdx );

		//�������� output_im_segIdxRGB�̍쐬
		cv::Mat  im_segIdxRGB;
		im_segIdxRGB = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		//�ђˁ@�ǉ��@-start
		cv::Point	cent[40]; //ID�Ƃ̏d�S�i�A�C�e��ID��40�j
		int			cent_sum[40];
		for( int j = 0; j < 40; j++ ){
			cent[j].x	= 0;
			cent[j].y	= 0;
			cent_sum[j] = 0;
		}
		//�ђˁ@�ǉ��@-end
		for( int j=0 ; j<im_segIdxRGB.rows ; j++ ){
			for( int i=0 ; i<im_segIdxRGB.cols ; i++ ){
				cv::Vec3b	c;
				int	id;
				id = im_objIdx.at<uchar>( j, i );
				//c��id�Ԗڂ̐F�����蓖�Ă�
				//c(0) = c(1) = c(2) = id; //���̍s��ύX

				//	�ђˁ@�ǉ��@-start
				c(0) = color40[id][2];
				c(1) = color40[id][1];
				c(2) = color40[id][0];

				//�d�S�_�Z�o�̂��߂̏���
				if( id != 0 ){ //�w�i�͏Ȃ�
					cent[id].x += i;
					cent[id].y += j;
					cent_sum[id]++;
				}
				//	�ђˁ@�ǉ��@-end

				im_segIdxRGB.at<cv::Vec3b>( j, i ) = c;
			}
		}
		//	�ђˁ@�ǉ��@-start
		//�d�S�_�Z�o
		for( int i = 0; i < 40; i++ ){
			if( cent_sum[i] != 0 ){
				cent[i].x = (int)( (double)cent[i].x / (double)cent_sum[i] );
				cent[i].y = (int)( (double)cent[i].y / (double)cent_sum[i] );
				char id_name[FNL];
				sprintf( id_name, "ID:%d", i );
				cv::putText( im_segIdxRGB, id_name, cent[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 3, CV_AA );
				cv::putText( im_segIdxRGB, id_name, cent[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255-color40[i][2], 255-color40[i][1], 255-color40[i][0]), 2, CV_AA );
			}
		}
		//�ђˁ@�ǉ��@-end

		cv::imwrite( "output_im_segIdxRGB.bmp", im_segIdxRGB );



		cv::Vec3b black;
		black(0) = black(1) = black(2) = 0;
		cv::Mat im_su_color = im_color.clone();
		for( int j=0 ; j<im_color.rows ; j++ ){
			for( int i=0 ; i<im_color.cols ; i++ ){
				if( im_cluster.at<uchar>( j, i ) == 0 ){
					im_su_color.at<cv::Vec3b>( j, i ) = black;
				}

			}
		}
		cv::imwrite( "result_cluster_color.bmp", im_su_color );
	}
	if( log2 ){
		//�c���_���X�g�̏o��(Valid)
		FILE	*fp_result;
		fp_result = fopen( "result_valid_grasp_points.txt", "w" );
		fprintf( fp_result,"i,j,label,segIdx,score,method\n" );
		for( int i=0 ; i<su.ind[0].chrom.size() ; i++ ){
			if( su.ind[0].chrom[i] ){
				fprintf( fp_result,"%d,%d,%d,%d,%.3lf", hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].label, hyp[i].segIdx, hyp[i].score);
				if( hyp[i].method == CNN_LABEL )       fprintf( fp_result,",CNN\n");
				else if( hyp[i].method == FAST_LABEL ) fprintf( fp_result,",Fast\n");
			}
		}
		fclose( fp_result );

		//�c���_���X�g�̏o��(All)
		fp_result = fopen( "result_all_grasp_points.txt", "w" );
		fprintf( fp_result,"i,j,label,segIdx,score,method\n" );
		for( int i=0 ; i<su.ind[0].chrom.size() ; i++ ){
			fprintf( fp_result,"%d,%d,%d,%d,%.3lf", hyp[i].grasp.x, hyp[i].grasp.y, hyp[i].label, hyp[i].segIdx, hyp[i].score);
			if( hyp[i].method == CNN_LABEL )       fprintf( fp_result,",CNN\n");
			else if( hyp[i].method == FAST_LABEL ) fprintf( fp_result,",Fast\n");
		}
		fclose( fp_result );  // 20160603 �쐼�C���u����Ă����Ȃ��`by�����Ђ낳��v
	}

	time2 = static_cast<double>(cv::getTickCount());
	Post_time = (time2 - time1)*1000 / cv::getTickFrequency();

	// �������Ԃ̕\��
	time_func_end = static_cast<double>(cv::getTickCount());
	double total_time;
	total_time = (time_func_end - time_func_start)*1000 / cv::getTickFrequency();
	fprintf( SE,"-------------------------------------------\n" );
	fprintf( SE,"Total time : %.1lf [msec]\n", total_time );
	fprintf( SE,"  Seg      : %.1lf [msec]\n", Seg_time );
	fprintf( SE,"  FAST     : %.1lf [msec]\n", FAST_time );
	fprintf( SE,"  FGE      : %.1lf [msec]\n", FGE_time );
	fprintf( SE,"  CNN      : %.1lf [msec]\n", CNN_time );
	fprintf( SE,"  SegAssign: %.1lf [msec]\n", Seg_assign_time );
	fprintf( SE,"  GA       : %.1lf [msec]\n", GA_time );
	fprintf( SE,"  Out      : %.1lf [msec]\n", Out_time );
	fprintf( SE,"  PostProc.: %.1lf [msec]\n", Post_time );
	fprintf( SE,"-------------------------------------------\n" );
	
	
	if( log ){
		fprintf( fp_logRecg,"-------------------------------------------\n" );
		fprintf( fp_logRecg,"Total time: %.1lf [msec]\n", total_time );
		fprintf( fp_logRecg,"  Seg      : %.1lf [msec]\n", Seg_time );
		fprintf( fp_logRecg,"  FAST     : %.1lf [msec]\n", FAST_time );
		fprintf( fp_logRecg,"  FGE      : %.1lf [msec]\n", FGE_time );
		fprintf( fp_logRecg,"  CNN      : %.1lf [msec]\n", CNN_time );
		fprintf( fp_logRecg,"  SegAssign: %.1lf [msec]\n", Seg_assign_time );
		fprintf( fp_logRecg,"  GA       : %.1lf [msec]\n", GA_time );
		fprintf( fp_logRecg,"  Out      : %.1lf [msec]\n", Out_time );
		fprintf( fp_logRecg,"  PostProc.: %.1lf [msec]\n", Post_time );
		fprintf( fp_logRecg,"-------------------------------------------\n" );
	}

	if( log2 ){
		FILE	*fp_time;
		fp_time = fopen( "result_time.txt", "w" );
		fprintf( fp_time,"-------------------------------------------\n" );
		fprintf( fp_time,"Total time: %.1lf [msec]\n", total_time );
		fprintf( fp_time,"  Seg      : %.1lf [msec]\n", Seg_time );
		fprintf( fp_time,"  FAST     : %.1lf [msec]\n", FAST_time );
		fprintf( fp_time,"  FGE      : %.1lf [msec]\n", FGE_time );
		fprintf( fp_time,"  CNN      : %.1lf [msec]\n", CNN_time );
		fprintf( fp_time,"  SegAssign: %.1lf [msec]\n", Seg_assign_time );
		fprintf( fp_time,"  GA       : %.1lf [msec]\n", GA_time );
		fprintf( fp_time,"  Out      : %.1lf [msec]\n", Out_time );
		fprintf( fp_time,"  PostProc.: %.1lf [msec]\n", Post_time );
		fprintf( fp_time,"-------------------------------------------\n" );
		fclose( fp_time );
	}

	if( log ) fclose( fp_logRecg );


	//���J�� �ȉ���2�s��ǉ� --2016.06.01
	hyp.clear();
	std::vector<Hyp> ().swap(hyp);
	


	return true;
}

//160602 �H���ǋL start
//// ����	cv::Mat	InImg			���͉摜(�f�v�X�摜)
////		int				PCASize			�@������Ɏg����`�T�C�Y
////		int				ResizeRate		�摜�����T�C�Y�̊��� (Size�� 1/ResizeRate �ɂȂ�)
////		int				BackGround		ID���_������ID
////		unsigned char	range_th		�����摜�̃����W���L���邽�߂̃p�����[�^�i0-255�j0������255����O (�����l�F140)
////		float			noise_theta_th	���ʔ��˃m�C�Y���������߂̃p�����[�^�i0.0-90.0�j�������قǃm�C�Y��������i�����l�F65.0�j
////		bool			log_img			log���o���L��



void calcNormalMap_recg( cv::Mat *InImg, int PCASize, int ResizeRate, int BackGroundID, unsigned char range_th, float noise_theta_th, bool log_img, cv::Mat *top, cv::Mat *front, cv::Mat *side ){

	// ���͉摜��tmp�ϐ��Ɋi�[
	cv::Mat InImg_tmp;			//���͂�tmp�摜
	cv::Mat top_tmp, front_tmp, side_tmp;	//top, front, side��tmp�摜
	InImg_tmp = InImg->clone();
	top_tmp = top->clone();
	front_tmp = front->clone();
	side_tmp = side->clone();

	// �T�C�Y�ϊ��摜��resize�ϐ��Ɋi�[
	cv::Mat InImg_resize;				//���͂�resize�摜
	cv::Mat top_resize, front_resize, side_resize;	//top, front, side��resize�摜
	cv::resize( InImg_tmp, InImg_resize, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST );
	cv::resize( top_tmp, top_resize, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST );
	cv::resize( front_tmp, front_resize, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST );
	cv::resize( side_tmp, side_resize, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST );

	// �f�o�b�O�p�ϐ��̏�����
	cv::Mat PlaneImg	= cv::Mat::zeros( InImg_resize.rows, InImg_resize.cols, CV_8UC3 ); //4���ʂɕ�����������
	cv::Mat ThetaImg	= cv::Mat::zeros( InImg_resize.rows, InImg_resize.cols, CV_8UC1 ); //�@���p�x����f�l�Ƃ����摜�i�O���[�X�P�[���j
	cv::Mat ThetaColorImg	= cv::Mat::zeros( InImg_resize.rows, InImg_resize.cols, CV_8UC3 ); //�@���p�x����f�l�Ƃ����摜�i�J���[�j
	cv::cvtColor( ThetaColorImg, ThetaColorImg, CV_BGR2HSV );

	// z�������̃����W��ύX(�΂ߎ��_�̂Ƃ��̂ݓ��삷��)
	if( BackGroundID%2 == 0 ){	//���_���΂߂ł����
	cv::Mat RangeImg = InImg_resize.clone(); 
		for( int j = 0; j < RangeImg.rows; j++ ){
			for( int i = 0; i < RangeImg.cols; i++ ){
				if( RangeImg.at<uchar>(j, i) < range_th  ){
					RangeImg.at<uchar>(j, i) = range_th-1;
				}
			}
		}
		cv::normalize( RangeImg, InImg_resize, 0, 255, cv::NORM_MINMAX );
		if(log_img) cv::imwrite( "im_range.bmp", RangeImg );
	}

	cv::Vec3b PlaneColor[4];
	PlaneColor[0] = cv::Vec3b(255, 255,   0);
	PlaneColor[1] = cv::Vec3b(  0, 255,   0);
	PlaneColor[2] = cv::Vec3b(  0, 255, 255);
	PlaneColor[3] = cv::Vec3b(  0,   0, 255);

	for( int j = 0; j < InImg_resize.rows; j++ ){
		for( int i = 0; i < InImg_resize.cols; i++ ){
			if( InImg_resize.at<uchar>(j, i) != 0 ){

				// �@������@====================================================================================================================
				//pca�Ώ۔͈͗p��Mat<pca_patch>���쐬
				cv::Mat pca_patch( PCASize*PCASize, 3, CV_16SC1); //-255����255�܂ł̒l�̊i�[���K�v
				int patch_count = 0;
				for( int jj = -(PCASize/2); jj <= (PCASize/2); jj++ ){
					for( int ii = -(PCASize/2); ii <= (PCASize/2); ii++ ){
						if( (0 < jj+j) && (0 < ii+i) && (jj+j < InImg_resize.rows) && (ii+i < InImg_resize.cols) ){	//�摜�O����o�邱�Ƃ�}��
							if( InImg_resize.at<uchar>(j+jj, i+ii) != 0 ){		//�v���l�����͂Ƃ΂�	
								pca_patch.at<short>(patch_count, 0) = ii;
								pca_patch.at<short>(patch_count, 1) = jj;
								pca_patch.at<short>(patch_count, 2) = InImg_resize.at<uchar>(j+jj, i+ii) - InImg_resize.at<uchar>(j, i);
								patch_count++;
							}else{
								pca_patch.at<short>(patch_count, 0) = NULL;
								pca_patch.at<short>(patch_count, 1) = NULL;
								pca_patch.at<short>(patch_count, 2) = NULL;
								patch_count++;
							}
						}else{
								pca_patch.at<short>(patch_count, 0) = NULL;
								pca_patch.at<short>(patch_count, 1) = NULL;
								pca_patch.at<short>(patch_count, 2) = NULL;
								patch_count++;

						}
					}
				}
				// pca_patch.resize( patch_count, patch_count ); 20160620 OpenCV2.1 �ł� pca_patch.resize���ł��Ȃ����߂ق��̏����ő�p

				if( patch_count > 10 ){ //PCA�̑Ώ۔͈͓��̓_�Q��10�ȉ��ł���΂Ƃ΂�
					cv::PCA pca( pca_patch, cv::Mat(), CV_PCA_DATA_AS_ROW, 3 ); //PCA�̎��s

					cv::Vec3f N_Vec; //�@��
					N_Vec.val[0] = pca.eigenvectors.at<float>(2, 0);	//�@����x����
					N_Vec.val[1] = pca.eigenvectors.at<float>(2, 1);	//�@����y����
					N_Vec.val[2] = pca.eigenvectors.at<float>(2, 2);	//�@����z����

					if( N_Vec.val[2] < 0 ){ //z�����������i���_�Ƌt�������Ă�����j
						N_Vec.val[0] = -N_Vec.val[0];
						N_Vec.val[1] = -N_Vec.val[1];
						N_Vec.val[2] = -N_Vec.val[2];
					}

				// ==============================================================================================================================

				// �o�͉摜�쐬�@================================================================================================================
					//20160622 �ђ˒ǉ��i�����������l���j -start
					cv::Vec3f VN_Vec;		// �J�������_���璍�ړ_�����ԒP�ʃx�N�g��
					float VN_VecSca;		// VN_Vec �̃m����
					float VN_dot;			// N_vec �� VN_Vec �Ƃ̓���
					float dot_theta;		// N_Vec �� VN_Vec ���Ȃ��p�x

					VN_Vec.val[0] = (InImg_resize.cols / 2) - i;
					VN_Vec.val[1] = (InImg_resize.rows / 2) - j;
					VN_Vec.val[2] = 500 - (InImg_resize.at<uchar>(j, i));
					VN_VecSca = sqrt( (double)VN_Vec.val[0]*VN_Vec.val[0] + (double)VN_Vec.val[1]*VN_Vec.val[1] + (double)VN_Vec.val[2]*VN_Vec.val[2] );
					VN_Vec.val[0] = VN_Vec.val[0] / VN_VecSca; 
					VN_Vec.val[1] = VN_Vec.val[1] / VN_VecSca; 
					VN_Vec.val[2] = VN_Vec.val[2] / VN_VecSca; 
					VN_dot = VN_Vec.dot(N_Vec);
					dot_theta = (acosf(VN_dot) / CV_PI) * 180.0;

					if( dot_theta < noise_theta_th ){

						//20160622 �ђ˒ǉ��i�����������l���j -end

						/*//20160622 �ђ˒ǉ��i�����������l���j�R�����g�A�E�g -start
						// 20160621 �ђ˒ǉ��i�����˃m�C�Y��}���j�@-start
						float thetaz_th	= 60.0;		// z �������̊p�x�����p�����[�^�i�������قǃm�C�Y�������₷���j 
						float thetay_th	= 30.0;		// y �������̊p�x�����p�����[�^�i�傫���قǃm�C�Y�������₷���j
						float thetaz, thetay;	
						thetaz = (acosf(N_Vec.val[2]) / CV_PI ) * 180.0;
						thetay = (acosf(fabs(N_Vec.val[1])) / CV_PI) * 180.0;

						//if( (thetaz_th < thetaz ) && (thetay < thetay_th) ){
						//}else{												
						*///20160622 �ђ˒ǉ��i�����������l���j�R�����g�A�E�g -end
						// 20160621 �ђ˒ǉ��i�����˃m�C�Y��}���j�@-end

						// �@�����摜���ʏ�ɓ��e�����p�x�irad�j���Z�o
						float rad;
						float N_Sxy;
						N_Sxy = sqrt( N_Vec.val[0]*N_Vec.val[0] + N_Vec.val[1]*N_Vec.val[1] );
						rad = acosf( (float)N_Vec.val[0]/(float)(N_Sxy+0.000001) );
						if( N_Vec.val[1] < 0 ){ //y������3,4�ی��ɂ�������
							rad = (M_PI*2.0) - rad;
						}

						if( log_img ){
							// HSV�F��Ԃ𗘗p����<ThetaColorImg>�Ƀ}�b�s���O
							// H:�@���p�x�@S:z�������@V:�Œ�l�i255�j
							ThetaColorImg.at<cv::Vec3b>(j, i).val[0] = (uchar)(((double)rad / ((double)M_PI*2.0)) * 180.0 + 0.5) ;
							ThetaColorImg.at<cv::Vec3b>(j, i).val[1] = (uchar)((N_Vec.val[2] * 255.0) + 0.5);
							ThetaColorImg.at<cv::Vec3b>(j, i).val[2] = 255;
						}

						if( log_img ){
							// �@���p�x�������g�����O���[�X�P�[���摜�쐬<ThetaImg>
							ThetaImg.at<uchar>(j, i) = (uchar)(((double)rad / ((double)M_PI*2.0)) * 255.0 + 0.5) ;
						}

						// 4�̕��ʂɊ��蓖�Ă�<PlaneImg>
						float	PlaneVec[4][3];		//4�̕��ʂ̖@���x�N�g��
						float	InPro;				//PlaneVec�Ƃ̓��ϒl
						float	InProMax = 0;		//InPro�̍ő�l
						int		PlaneID  = 0;		//����ID(0:�O�ʁ@1:��ʁ@2:���ʁ@4:�E��)

						if( BackGroundID%2 == 0 ){	//���_���΂߂ł����
							PlaneVec[0][0] =  0.0; PlaneVec[0][1] =  0.70710; PlaneVec[0][2] = 0.70710;	//�O��
							PlaneVec[1][0] =  0.0; PlaneVec[1][1] = -0.70710; PlaneVec[1][2] = 0.70710;	//���
							PlaneVec[2][0] = -1.0; PlaneVec[2][1] =  0.0;	  PlaneVec[2][2] =  0.0;	//����
							PlaneVec[3][0] =  1.0; PlaneVec[3][1] =  0.0;	  PlaneVec[3][2] =  0.0;	//�E��
						}else{						//���_�����ʂł����
							PlaneVec[0][0] =  0.0; PlaneVec[0][1] =  0.0; PlaneVec[0][2] = 1.0;	//�O��
							PlaneVec[1][0] =  0.0; PlaneVec[1][1] = -1.0; PlaneVec[1][2] = 0.0;	//���
							PlaneVec[2][0] = -1.0; PlaneVec[2][1] =  0.0; PlaneVec[2][2] = 0.0;	//����
							PlaneVec[3][0] =  1.0; PlaneVec[3][1] =  0.0; PlaneVec[3][2] = 0.0;	//�E��
						}

						for( int k = 0; k < 4; k++ ){
							InPro = 0.0;
							for( int l = 0; l < 3; l++ ){
								InPro += (float)PlaneVec[k][l]*(float)N_Vec.val[l];
							}
							if( InProMax < InPro ){
								InProMax = InPro;
								PlaneID  = k;
							}
						}
						if(log_img){
							PlaneImg.at<cv::Vec3b>(j, i) = PlaneColor[PlaneID];
						}
					
						// <top> <front> <side> �̍쐬
						if( PlaneID == 0 )	front_resize.at<uchar>(j, i)	= 255;
						else if( PlaneID == 1 ) top_resize.at<uchar>(j, i)	= 255;
						else if( PlaneID == 2 ) side_resize.at<uchar>(j, i)	= 255;
						else if( PlaneID == 3 ) side_resize.at<uchar>(j, i)	= 255;

					}// 20160621 �ђ˒ǉ��i�����˃m�C�Y��}���j

				}
				// �o�͉摜�쐬�@================================================================================================================

			}
		}
	}

	//top, front, side �摜�� resize
	cv::resize( top_resize, *top, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST );
	cv::resize( front_resize, *front, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST );
	cv::resize( side_resize, *side, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST );

	if( log_img ){
		//�f�o�b�O�p�ϐ��� resize �� imwrite
		cv::Mat ThetaColorImg_ans;
		cv::Mat PlaneImg_ans;
		cv::Mat ThetaImg_ans;
		cv::cvtColor( ThetaColorImg, ThetaColorImg, CV_HSV2BGR );
		cv::resize( ThetaColorImg, ThetaColorImg_ans, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST );
		cv::resize( ThetaImg, ThetaImg_ans, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST );
		cv::resize( PlaneImg, PlaneImg_ans, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST );
		cv::imwrite( "ThetaColorImg.bmp", ThetaColorImg_ans );
		cv::imwrite( "Plane.bmp", PlaneImg_ans );
		cv::imwrite( "Theta.bmp", ThetaImg_ans );
		//�o�͉摜���w��i���݂�<ThetaImg>�j
		//*OutImg = ThetaImg_ans; 
	}

}
// �ђˁ@�ǉ��@-end

//160602 �H���ǋL end
