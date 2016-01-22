/////////////////////////////////////////////////////////////////////////////
//
//	librecglib.cpp:
//
//  Shuichi AKIZUKI
//
//	(C) 2015 ISL, Chukyo University All rights reserved.
//
//  Note:
//		��w���F�������v���O�����̋��ʓI�Ȋ֐��Q�̓��e
//	
//		2015.04.20
//		�R�����g�����������ǉ��D
//		2015.04.30
//		���c�N�̃R�[�h�𔽉f�D
//
//////////////////////////////////////////////////////////////////////////////

//#include "stdafx.h"

#include "recg.h"
#include "librecglib.h"
#include "bin_size.h"
#include "algorithm_table.h"

using namespace cv;

#define seg_noise_reduction (0) //�Z�O�����e�[�V�������g�����m�C�Y������ON/OFF


void ShowRecognitionResult( int method, double *workPos, double *score ){

	if( method == METHOD_WHF ){
		fprintf( SE,"\n" );
		fprintf( SE,"/--------------------------------------------/\n" );
		fprintf( SE,"Result of Weighted Hough Forest\n" );

		fprintf( SE,"Work Position:\n" );
		fprintf( SE,"( " );
		for( int i=0 ; i<12 ; i++ ){
			fprintf( SE,"%.1lf ", workPos[i] );
		}
		fprintf( SE,")\n" );

	}else if( method == METHOD_VPM ){
		fprintf( SE,"\n" );
		fprintf( SE,"/--------------------------------------------/\n" );
		fprintf( SE,"Result of Vector Pair Matching\n" );
		fprintf( SE,"Work Position:\n" );
		fprintf( SE,"    | %.3lf %.3lf %.3lf |\n", workPos[0], workPos[1], workPos[2] );
		fprintf( SE,"R = | %.3lf %.3lf %.3lf |\n", workPos[3], workPos[4], workPos[5] );
		fprintf( SE,"    | %.3lf %.3lf %.3lf |\n", workPos[6], workPos[7], workPos[8] );
		fprintf( SE,"t = < %.3lf %.3lf %.3lf >\n", workPos[9], workPos[10], workPos[11] );

		fprintf( SE,"( " );
		for( int i=0 ; i<12 ; i++ ){
			fprintf( SE,"%.1lf ", workPos[i] );
		}
		fprintf( SE,")\n" );

	}else if( method == METHOD_SIMPLE ){
		fprintf( SE,"\n" );
		fprintf( SE,"/--------------------------------------------/\n" );
		fprintf( SE,"Result of Simple method (SPRH)\n" );
		fprintf( SE,"Work Position:\n" );
		fprintf( SE,"( " );
		for( int i=0 ; i<12 ; i++ ){
			fprintf( SE,"%.1lf ", workPos[i] );
		}
		fprintf( SE,")\n" );
	}
	fprintf( SE,"Score: %lf\n", *score );
	fprintf( SE,"/--------------------------------------------/\n" );
	fprintf( SE,"\n" );


}

void CopyWorkPos( double *workPos1, int num, double *workPos2 ){

	for( int i=0 ; i<num ; i++ ){
		workPos2[i] = workPos1[i];
	}

}

bool RecgWeightedHoughForest(int *binNum, int *itemIdx, cv::Mat color, cv::Mat Sub, double *workPos, double *score, std::vector<int>& WHFClusterIdx ){
	//�֐���void�^����bool�^�ɕύX+ cv::Mar Sub�� WHF_cluster_idx �������ɒǉ� 2015.04.26���c

		int bin,itemNum;
		int *ref_num;
		int refNum;

		cv::Mat testImage;
		//1�`�����l���̃O���[�X�X�P�[���摜���쐬
		cv::cvtColor(color,testImage,CV_BGR2GRAY,1);
		bin=*binNum;
		itemNum=*itemIdx;

		//�����㏑��������Ă��܂��̂Ő[���R�s�[
		cv::Mat Sub2 = Sub.clone();
		cv::Mat opening = Sub.clone();
		Detector detector;
		//���͉摜�̃X�P�[����WHFcommon.h�Œ�`����SCALE�̒l�ɕϊ�(�`�������W�����̃X�P�[���ɂ��킹�Ē���)
		resize(testImage, testImage, Size() , SCALE, SCALE, INTER_CUBIC);
		resize(opening, opening, Size() , SCALE, SCALE, INTER_CUBIC);
		cv::erode(opening, opening, cv::Mat(), cv::Point(-1,-1), 2);//�c�����k�ɂ��m�C�Y�E���̍팸 ���c
		cv::dilate(opening, opening, cv::Mat(), cv::Point(-1,-1), 4);
		//cv::erode(opening, opening, cv::Mat(), cv::Point(-1,-1), 1);
		//���o�p�֐�
		detector.computeError(bin,itemNum,testImage,opening,workPos,score,ref_num);//���t�@�����X�ԍ���Ԃ��悤�ɕύX�@���c


		//���t�@�����X�̊m�F �|�C���^�^�ł��B���c
		std::cout<<"ref"<<*ref_num<<std::endl;;

		//LOG�����o���̔���
		#ifdef LOGMODE
		std::ofstream ofs( "./WHF_LOG.txt" ,std::ios::app);
		ofs<<"binNum="<<bin<<"\titemIdx="<<itemNum<<"\tworkPos="<<workPos[0]<<","<<workPos[1]<<"\tscore="<<*score<<"\tref_num="<<*ref_num<<std::endl;
		ofs.close();
		#endif
		
		cv::Mat WHFCluster( HEIGHT, WIDTH, CV_8UC1, cv::Scalar(0) );
		//�Z�O�����e�[�V����
		refNum = *ref_num;

		//���͕�����������(���q)
		WHFCluster = item_segmentation(Sub, (int)workPos[0], (int)workPos[1], itemNum, refNum);

		int	cnt;
		cnt =0;
		for( int i=0 ; i<WIDTH*HEIGHT ; i++ ){
			if(	Sub2.data[i] != 0 ){
				if( WHFCluster.data[i] != 0 ){
					WHFClusterIdx.push_back( cnt );
				}
				cnt++;
			}
		}

		cv::imwrite("function_test.bmp", WHFCluster);


		return true;
}

bool Recg( int **binIdx, int *binNum, int *itemIdx, double *pnt, unsigned char *depth, unsigned char *color, int *flag, 
		   double *workPos, unsigned char *cluster ){

	// ���S���W�̏�����
	workPos[0] = -1;  //��O�l������
	workPos[1] = -1;  //�F���ł��Ă���X�V�����
	*flag = -1;


	FILE		*fp;
	char		funcname[256], name[256];
	double		deg2rad, rad2deg;

	deg2rad = M_PI/180.0;
	rad2deg = 180.0/M_PI;

	strcpy( funcname,"Recg()" );
	if ((fp = fopen("log_Recg.txt", "w")) == NULL) {
		fprintf( SE, "!!Error!! %s\n", funcname );
		fprintf( SE, "  ���O�t�@�C���̏����o�����ł��܂���D\n");
		return false;
	}


	// Bin �̏��̕\��
	fprintf( fp,"binIdx:\n");
	for( int j=0 ; j<12 ; j++ ){
		fprintf( fp,"%2d: ", j );
		for( int i=0 ; i<10 ; i++ ){
			fprintf( fp,"%d ", binIdx[j][i] );
		}
		fprintf( fp,"\n" );
	}
	fprintf( fp,"binNum: %d\n", *binNum );
	fprintf( fp,"itemIdx: %d\n", *itemIdx );
	fprintf( fp,"flag: %d\n", *flag );

	//=========================================================================//
	// �O�����D
	// �r���̓����̃f�[�^�̎��o���CRGB���t���_�Q�f�[�^�̐����D
	//=========================================================================//


	// depth�摜�̐���
	cv::Mat d_img;
	d_img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	for( int i=0 ; i<WIDTH*HEIGHT; i++ ){
		d_img.at<uchar>( i / WIDTH, i % WIDTH ) = depth[i];
	}


	cv::Mat median_img;
	cv::medianBlur( d_img, median_img, 3 );
	for( int i=0 ; i<4 ; i++ ){
		cv::medianBlur( median_img, median_img, 3 );
	}

	cv::Mat im_label_mask;
	im_label_mask = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	SegmentationNoiseReduction(median_img, im_label_mask );



	// �f�[�^�ϊ��D���v���_���������_�̃C���f�N�X��PntIdx�ɓ����Ă���D
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> PntIdx;
	if(seg_noise_reduction == 1){
		Pnt2PCDXYZ( pnt, im_label_mask, WIDTH*HEIGHT, cloud, PntIdx );
	}else{
		Pnt2PCDXYZ( pnt, median_img, WIDTH*HEIGHT, cloud, PntIdx );
	}
	// �r���̃T�C�Y�p�����[�^�̓ǂݍ��݁D
	//FILE				*fp_bin;
	//char				dum[256];
	//char				bin_param_fname[FNL];
	//errno_t				error_check;
	//fprintf( fp,"Loading bin parameter\n" );
	//sprintf( bin_param_fname,"%s", BIN_PARAM_NAME );
	//if( (error_check = fopen_s( &fp_bin, bin_param_fname, "r" )) != 0 ){
	//	fprintf( fp,"!!Error %s cannot be load.\n", bin_param_fname );
	//	return false;
	//};

	//20150429-�H���ǋL������
	// �r�����̃f�[�^�̎��o���D
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sub (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> SubIdx;
	//BinRemove( cloud, dist_cam2bin[*binNum], cam_inclination, bin_width[*binNum], cloud_sub, SubIdx );
	//20150429-�H���ǋL������
	
	//20150518-�H���ǋL������
	//�r�������������g���ăf�[�^������
	BinRemove2( cloud, bin_width[*binNum], cloud_sub, SubIdx );
	//BinAndNoiseRemove( cloud, bin_width[*binNum], d_img, cloud_sub, SubIdx );
	//20150518-�H���ǋL������


	// �w�i�����摜�̍쐬�D
	cv::Mat Sub_img; //
	 Sub_img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	for( int i=0 ; i<SubIdx.size() ; i++ ){
		unsigned char tmp;
		tmp = depth[ PntIdx[SubIdx[i]] ];
		 Sub_img.at<uchar>( PntIdx[SubIdx[i]] / WIDTH, PntIdx[SubIdx[i]] % WIDTH ) = tmp;
	}

	// 20150521 �H���ǋL�Dstart
	//�摜�\����
	cv::Mat	im_orgMedian;  //�������������͉摜
	cv::Mat im_edgeFlag;   //�G�b�W�t���O

	//�p�����[�^
	int m_nPrmCannyEdge;
	int m_nPrmMedFilter;

	m_nPrmCannyEdge		=	80;		// �L���j�[�G�b�W
	m_nPrmMedFilter		=	9;		// ���f�B�A���t�B���^
	// (004) ���͋����摜�������p�������փR�s�[
	//im_orgMedian	=	Sub_img.clone();
	im_orgMedian	=	median_img.clone();
	// (005) �����p�摜�ɂQ�񃁃f�B�A���t�B���^�����ĕ�����
	for(int i = 0; i < 2; i++)
	{
		medianBlur(im_orgMedian, im_orgMedian, m_nPrmMedFilter);
	}

#if OUTPUT3
	cv::imshow( "im_orgMedian  ������", im_orgMedian );
	cv::imwrite( "segclass_im_orgMedian.bmp", im_orgMedian );
	cv::waitKey(0);
#endif

	// (014) �G�b�W���o(Canny) 
	cv::Mat im_tmp1;
	cv::Canny( im_orgMedian, im_tmp1, 0, m_nPrmCannyEdge, 3);

#if OUTPUT3
	cv::imshow( "im_tmp �G�b�W���o", im_tmp1 );
	cv::imwrite( "segclass_edge.bmp", im_tmp1 );
	cv::waitKey(0);
#endif

	// (015) �G�b�W�c�� im_tmp1 -> im_tmp1
	cv::Mat C	=	cv::Mat::ones(3, 3, CV_32FC1);
	cv::dilate(im_tmp1, im_edgeFlag, C);

#if OUTPUT3
	cv::imshow( "im_edgeFlag �G�b�W�c��", im_edgeFlag );
	cv::imwrite( "segclass_dilated_edge.bmp", im_edgeFlag );
	cv::waitKey(0);
#endif


	// �f�[�^�ϊ��D���v���_���������_�̃C���f�N�X��PntIdx�ɓ����Ă���D
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> PntEdgeIdx;
	Pnt2PCDXYZ_Edge( pnt, im_edgeFlag, WIDTH*HEIGHT, cloud_edge, PntEdgeIdx );
	//Pnt2PCDXYZ( pnt, im_tmp1, WIDTH*HEIGHT, cloud_edge, PntEdgeIdx );
#if OUTPUT2
	if( CheckSavePCD( cloud_edge ) == true ){
		pcl::io::savePCDFile( "cloud_edge.pcd", *cloud_edge ); 
	}
#endif
	// 20150521 �H���ǋL�Dend

	
	// �V�[���摜�̍쐬�iRGB�j //WHF�EFAST�ɓ��͂��邽�߂�����Ɉړ��@15.5.10���c
	cv::Mat color_img; // �V�[���摜�iRGB�j
	color_img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
	cv::Vec3b tmp;
	for( int i=0 ; i<WIDTH*HEIGHT ; i++ ){
		tmp(0) = color[ 3*i ];
		tmp(1) = color[ 3*i+1 ];
		tmp(2) = color[ 3*i+2 ];
		color_img.at<cv::Vec3b>( i / WIDTH, i % WIDTH ) = tmp;
	}

	// �}�X�N�������͉摜�̍쐬�D
	Mat masked_img( HEIGHT, WIDTH, CV_8UC3 );
	for( int i=0 ; i<SubIdx.size() ; i++ ){
		tmp(0) = color[ 3*PntIdx[SubIdx[i]] ];
		tmp(1) = color[ (3*PntIdx[SubIdx[i]])+1 ];
		tmp(2) = color[ (3*PntIdx[SubIdx[i]])+2 ];
		masked_img.at<cv::Vec3b>( PntIdx[SubIdx[i]] / WIDTH, PntIdx[SubIdx[i]] % WIDTH ) = tmp;
	}




	// Simple �A���S���Y���p�̏���
	// RGB�����_�Q�̐���
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_subRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_subRGB->width = cloud_sub->width;
	cloud_subRGB->height = cloud_sub->height;
	cloud_subRGB->points.resize( cloud_sub->points.size() );
	for( int i=0 ; i<cloud_sub->points.size() ; i++ ){
		cloud_subRGB->points[i].x = cloud_sub->points[i].x;
		cloud_subRGB->points[i].y = cloud_sub->points[i].y;
		cloud_subRGB->points[i].z = cloud_sub->points[i].z;
		cloud_subRGB->points[i].r = color[ 3*PntIdx[SubIdx[i]]+2 ];
		cloud_subRGB->points[i].g = color[ 3*PntIdx[SubIdx[i]]+1 ];
		cloud_subRGB->points[i].b = color[ 3*PntIdx[SubIdx[i]] ];
	}


	// ���ꂼ��̃A���S���Y���ł̔F��
	// ���ׂē��삳���āC�ł��������̂��o�͂�����D
	// �V���v���A���S���Y�����P�[�X���Ƃɏ����� 2015.05.06 ����
	double				SimpleScore, SimpleScore2_1, SimpleScore2_2, SimpleScore2_3;
	double				VPMScore, VPMScore2;
	double				WHFScore, WHFScore2;
	double				FASTScore2;
	double				SimpleworkPos[12];
	double				VPMworkPos[12];
	double				WHFworkPos[12];
	double              FASTworkPos[12];
	int					SimpleCenterIdx1, SimpleCenterIdx2, SimpleCenterIdx3;
	int					VPMCenterIdx;
	std::vector<int>	SimpleClusterIdx1, SimpleClusterIdx2, SimpleClusterIdx3;


	int					Error;
	double				Score;
	std::vector<int>	ClusterIdx;

	Error = 1;
	Score = DBL_MAX;

	int					SimpleError1, SimpleError2, SimpleError3, FASTError; //�G���[�̃t���O
	SimpleError1 = SimpleError2 = SimpleError3 = 1; //0:�G���[�Ȃ��D1:�G���[����D
	FASTError = 1; //���̕ϐ���default��1�ɂ��Ă����D�F���ł����ꍇ��0�ɂȂ�܂��D

	// �X�R�A�C�p���f�[�^�̏�����
	SimpleScore = VPMScore = WHFScore = 0.0;
	SimpleScore2_1 = SimpleScore2_2 = SimpleScore2_3 = VPMScore2 = WHFScore2 = FASTScore2 = DBL_MAX;
	for( int i=0 ; i<12 ; i++ ){
		SimpleworkPos[i] = 0.0;
		VPMworkPos[i] = 0.0;
		WHFworkPos[i] = 0.0;
		FASTworkPos[i] = 0.0;
	}
	VPMworkPos[0] = VPMworkPos[4] = VPMworkPos[8] = 1.0;

	// �r���̎�ނ����� 2015.05.06 ����
	int num_of_seg ;
	num_of_seg = 0 ;
	for( int i=0 ; i<10 ; i++ ) {
		//fprintf( stderr, "binIdx= %d\n", binIdx[i] ) ;
		if( 0<binIdx[*binNum][i] && binIdx[*binNum][i]<26 )	num_of_seg++ ;
	}

	//20150518-�H���ǋL������
	// 0->single bin,  1->double bin,  2->multi bin
	int bin_condition;
	if( num_of_seg == 1 ){
		bin_condition = 0;
	}else if( num_of_seg == 2 ){
		bin_condition = 1;
	}else{
		bin_condition = 2;
	}
	//20150518-�H���ǋL������

	//=========================================================================//
	// �F������
	//=========================================================================//
	
	//20150518-�H���ǋL������

	//=========================================================================//
	// Cascated FAST �����܂����������ȕ��̂Ɋւ��Ă͗D��I�ɓ�����
	//=========================================================================//
	if( C_FAST_SW[*itemIdx] == ON ){
		// Cascaded Fast
		if(RecgCascadedFast(itemIdx, color_img, Sub_img, ClusterIdx, FASTworkPos )==false){
			fprintf( fp,"!!Error has been occured in RecgCascadedFast.\n" );
			fprintf( fp,"  ���̃G���[��FAST�ŔF���ł��Ȃ������ꍇ���܂݂܂�.\n" );
			FASTError = 1;
		}else{
			fprintf( fp,"Cascaded Fast End\n" );
			FASTError = 0;
			workPos[0] = FASTworkPos[0];
			workPos[1] = FASTworkPos[1];
			*flag = METHOD_FAST;
		}
	}
	//=========================================================================//
	// Cascated FAST end
	//=========================================================================//



	//=========================================================================//
	// WHF or Simple or VPM �ɂ��}�b�`���O start
	//=========================================================================//
	if( FASTError ){ //FAST�ŔF���ł������ǂ����`�F�b�N�D
		if( method[*itemIdx][bin_condition] == METHOD_SIMPLE ){
			// Simple �Z�O�����e�[�V�����ɂ��}�b�`���O
			*flag = METHOD_SIMPLE;
			fprintf( fp,"RecgSimple Start1\n" );
			if( RecgSimple( num_of_seg, itemIdx, cloud_subRGB, SimpleworkPos, &SimpleScore, &SimpleCenterIdx1, SimpleClusterIdx1 ) == false ){
				fprintf( fp,"!!Error has been occured in RecgSimple().\n" );
				SimpleScore  = 0.0;
				SimpleError1 = 1;
			}else{
				SimpleError1 = 0;
			}
			fprintf( fp,"RecgSimple End1\n" );
			num_of_seg += 1 ;
			fprintf( fp,"RecgSimple Start2\n" );
			if( RecgSimple( num_of_seg, itemIdx, cloud_subRGB, SimpleworkPos, &SimpleScore, &SimpleCenterIdx2, SimpleClusterIdx2 ) == false ){
				fprintf( fp,"!!Error has been occured in RecgSimple().\n" );
				SimpleScore  = 0.0;
				SimpleError2 = 1;
			}else{
				SimpleError2 = 0;
			}
			fprintf( fp,"RecgSimple End2\n" );
			num_of_seg += 1 ;
			fprintf( fp,"RecgSimple Start3\n" );
			if( RecgSimple( num_of_seg, itemIdx, cloud_subRGB, SimpleworkPos, &SimpleScore, &SimpleCenterIdx3, SimpleClusterIdx3 ) == false ){
				fprintf( fp,"!!Error has been occured in RecgSimple().\n" );
				SimpleScore  = 0.0;
				SimpleError3 = 1;
			}else{
				SimpleError3 = 0;
			}
			fprintf( fp,"RecgSimple End3\n" );

			// 3�̒��ł������̂�I�Ԃ��߂ɃX�R�A���Z�o
			rts_Score(itemIdx,color, PntIdx, SubIdx, SimpleClusterIdx1, &SimpleScore2_1);
			rts_Score(itemIdx,color, PntIdx, SubIdx, SimpleClusterIdx2, &SimpleScore2_2);
			rts_Score(itemIdx,color, PntIdx, SubIdx, SimpleClusterIdx3, &SimpleScore2_3);
			fprintf( fp,"SimpleScore2_1: %lf\n", SimpleScore2_1 );
			fprintf( fp,"SimpleScore2_2: %lf\n", SimpleScore2_2 );
			fprintf( fp,"SimpleScore2_3: %lf\n", SimpleScore2_3 );

			if( (SimpleScore2_2 <= SimpleScore2_1) && (SimpleScore2_3 <= SimpleScore2_1) ){ //Simple�P�[�X1�̃X�R�A���ő�
				*flag = METHOD_SIMPLE;
				//�N���X�^���_�Q�̃R�s�[
				if( SimpleError1 == 0 ){
					workPos[0] = PntIdx[SubIdx[SimpleCenterIdx1]] % WIDTH;
					workPos[1] = PntIdx[SubIdx[SimpleCenterIdx1]] / WIDTH;
					for( int i=0 ; i<SimpleClusterIdx1.size() ; i++ ){
						ClusterIdx.push_back( SimpleClusterIdx1[i] );
					}
				}
				Error = 0;
				fprintf( fp,"Cluster = Simple1\n" );
			}else if( (SimpleScore2_1 < SimpleScore2_2) && (SimpleScore2_3 <= SimpleScore2_2) ){ //Simple�P�[�X2�̃X�R�A���ő�
				*flag = METHOD_SIMPLE;
				//�N���X�^���_�Q�̃R�s�[
				if( SimpleError2 == 0 ){
					workPos[0] = PntIdx[SubIdx[SimpleCenterIdx2]] % WIDTH;
					workPos[1] = PntIdx[SubIdx[SimpleCenterIdx2]] / WIDTH;
					for( int i=0 ; i<SimpleClusterIdx2.size() ; i++ ){
						ClusterIdx.push_back( SimpleClusterIdx2[i] );
					}
				}
				Error = 0;
				fprintf( fp,"Cluster = Simple2\n" );
			}else{ //Simple�P�[�X3�̃X�R�A���ő�
				*flag = METHOD_SIMPLE;
				//�N���X�^���_�Q�̃R�s�[
				if( SimpleError3 == 0 ){
					workPos[0] = PntIdx[SubIdx[SimpleCenterIdx3]] % WIDTH;
					workPos[1] = PntIdx[SubIdx[SimpleCenterIdx3]] / WIDTH;
					for( int i=0 ; i<SimpleClusterIdx3.size() ; i++ ){
						ClusterIdx.push_back( SimpleClusterIdx3[i] );
					}
				}
				Error = 0;
				fprintf( fp,"Cluster = Simple3\n" );
			}

		}else if( method[*itemIdx][bin_condition] == METHOD_WHF ){
			// Weighted Hough Forest �ɂ��}�b�`���O
			*flag = METHOD_WHF;
			fprintf( fp,"RecgWeightedHoughForest Start\n" );
			if( RecgWeightedHoughForest( binNum, itemIdx, masked_img,Sub_img, workPos, &Score, ClusterIdx ) == false ){
				fprintf( fp,"!!Error has been occured in RecgWeightedHoughForest.\n" );
				Score = 0.0;
				Error = 1;
			}else{
				Error = 0;
			}
			fprintf( fp,"RecgWeightedHoughForest End\n" );
		}else{
			// VPM �ɂ��}�b�`���O
			fprintf( fp,"RecgVPM Start\n" );
			*flag = METHOD_VPM;
			if( RecgVPM( itemIdx, cloud_sub, cloud_edge, workPos, &Score, &VPMCenterIdx, ClusterIdx ) == false ){
				fprintf( fp,"!!Error has been occured in RecgVPM().\n" );
				Score = 0.0;
				Error = 1;
			}else{
				workPos[0] = PntIdx[SubIdx[VPMCenterIdx]] % WIDTH;
				workPos[1] = PntIdx[SubIdx[VPMCenterIdx]] / WIDTH;
				Error = 0;
			}
			fprintf( fp,"RecgVPM End\n" );
		}
	}
	//=========================================================================//
	// WHF or Simple or VPM �ɂ��}�b�`���O end
	//=========================================================================//




	//=========================================================================//
	// �N���X�^�f�[�^�̃R�s�[�@start
	//=========================================================================//
	fprintf( fp,"Make cluster...\n" );
	if( (Error == 0) || (FASTError==0) ){
		for( int i=0 ; i<ClusterIdx.size() ; i++ ){
			cluster[ PntIdx[SubIdx[ClusterIdx[i]]] ] = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
		}
		//*flag = 1;
	}else{ // �G���[�����D�����f�[�^�����̂܂܃R�s�[
		for( int i=0 ; i<WIDTH*HEIGHT ; i++ ){
			cluster[ i ] = depth[ i ];
		}
		fprintf( fp,"Cluster = No.\n" );
		*flag = METHOD_ERROR;
	}
	fprintf( fp,"Make cluster...End\n" );
	//=========================================================================//
	// �N���X�^�f�[�^�̃R�s�[�@end
	//=========================================================================//

	fprintf( SE,"Error         = %d\n", Error );
	fprintf( SE,"Simple Error1 = %d\n", SimpleError1 );
	fprintf( SE,"Simple Error2 = %d\n", SimpleError2 );
	fprintf( SE,"Simple Error3 = %d\n", SimpleError3 );
	fprintf( SE,"FASTError     = %d\n", FASTError );


#if OUTPUT3
	//=========================================================================//
	// �f�o�b�O�f�[�^�̏o��
	//=========================================================================//

	// �N���X�^�摜�̍쐬�D
	int imgX, imgY;
	if( (method[*itemIdx][bin_condition] == METHOD_VPM) && (Error == 0) ){
		cv::Mat cluster_img_vpm; // VPM
		cluster_img_vpm = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		for( int i=0 ; i<ClusterIdx.size() ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			tmp(1) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			tmp(2) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			cluster_img_vpm.at<cv::Vec3b>( PntIdx[SubIdx[ClusterIdx[i]]] / WIDTH, PntIdx[SubIdx[ClusterIdx[i]]] % WIDTH ) = tmp;
		}
		imgX = PntIdx[SubIdx[VPMCenterIdx]] % WIDTH;
		imgY = PntIdx[SubIdx[VPMCenterIdx]] / WIDTH;
		//cv::circle( cluster_img_vpm, cv::Point(imgX, imgY), 6, cv::Scalar(0,0,255), -1, CV_AA, 0 );
		//cv::resize( cluster_img_vpm, cluster_img_vpm, cv::Size() , SCALE, SCALE, INTER_CUBIC);

		cv::imwrite( "cluster_img_vpm.bmp", cluster_img_vpm );
	}

	// �V���v���A���S���Y�����P�[�X���Ƃɕۑ�
	if( (method[*itemIdx][bin_condition] == METHOD_SIMPLE) && (SimpleError1 == 0) ){
		cv::Mat cluster_img_simple; // Simple
		cluster_img_simple = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		for( int i=0 ; i<SimpleClusterIdx1.size() ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = depth[ PntIdx[SubIdx[SimpleClusterIdx1[i]]] ];
			tmp(1) = depth[ PntIdx[SubIdx[SimpleClusterIdx1[i]]] ];
			tmp(2) = depth[ PntIdx[SubIdx[SimpleClusterIdx1[i]]] ];
			cluster_img_simple.at<cv::Vec3b>( PntIdx[SubIdx[SimpleClusterIdx1[i]]] / WIDTH, PntIdx[SubIdx[SimpleClusterIdx1[i]]] % WIDTH ) = tmp;
		}
		imgX = PntIdx[SubIdx[SimpleCenterIdx1]] % WIDTH;
		imgY = PntIdx[SubIdx[SimpleCenterIdx1]] / WIDTH;
		//cv::circle( cluster_img_simple, cv::Point(imgX, imgY), 6, cv::Scalar(0,0,255), -1, CV_AA, 0 );
		//cv::resize( cluster_img_simple, cluster_img_simple, cv::Size() , SCALE, SCALE, INTER_CUBIC);
		cv::imwrite( "cluster_img_simple1.bmp", cluster_img_simple );
	}

	if( (method[*itemIdx][bin_condition] == METHOD_SIMPLE) && (SimpleError2 == 0) ){
		cv::Mat cluster_img_simple; // Simple
		cluster_img_simple = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		for( int i=0 ; i<SimpleClusterIdx2.size() ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = depth[ PntIdx[SubIdx[SimpleClusterIdx2[i]]] ];
			tmp(1) = depth[ PntIdx[SubIdx[SimpleClusterIdx2[i]]] ];
			tmp(2) = depth[ PntIdx[SubIdx[SimpleClusterIdx2[i]]] ];
			cluster_img_simple.at<cv::Vec3b>( PntIdx[SubIdx[SimpleClusterIdx2[i]]] / WIDTH, PntIdx[SubIdx[SimpleClusterIdx2[i]]] % WIDTH ) = tmp;
		}
		imgX = PntIdx[SubIdx[SimpleCenterIdx2]] % WIDTH;
		imgY = PntIdx[SubIdx[SimpleCenterIdx2]] / WIDTH;
		//cv::circle( cluster_img_simple, cv::Point(imgX, imgY), 6, cv::Scalar(0,0,255), -1, CV_AA, 0 );
		//cv::resize( cluster_img_simple, cluster_img_simple, cv::Size() , SCALE, SCALE, INTER_CUBIC);
		cv::imwrite( "cluster_img_simple2.bmp", cluster_img_simple );
	}

	if( (method[*itemIdx][bin_condition] == METHOD_SIMPLE) && (SimpleError3 == 0) ){
		cv::Mat cluster_img_simple; // Simple
		cluster_img_simple = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		for( int i=0 ; i<SimpleClusterIdx3.size() ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = depth[ PntIdx[SubIdx[SimpleClusterIdx3[i]]] ];
			tmp(1) = depth[ PntIdx[SubIdx[SimpleClusterIdx3[i]]] ];
			tmp(2) = depth[ PntIdx[SubIdx[SimpleClusterIdx3[i]]] ];
			cluster_img_simple.at<cv::Vec3b>( PntIdx[SubIdx[SimpleClusterIdx3[i]]] / WIDTH, PntIdx[SubIdx[SimpleClusterIdx3[i]]] % WIDTH ) = tmp;
		}
		imgX = PntIdx[SubIdx[SimpleCenterIdx3]] % WIDTH;
		imgY = PntIdx[SubIdx[SimpleCenterIdx3]] / WIDTH;
		//cv::circle( cluster_img_simple, cv::Point(imgX, imgY), 6, cv::Scalar(0,0,255), -1, CV_AA, 0 );
		//cv::resize( cluster_img_simple, cluster_img_simple, cv::Size() , SCALE, SCALE, INTER_CUBIC);
		cv::imwrite( "cluster_img_simple3.bmp", cluster_img_simple );
	}


	if( (method[*itemIdx][bin_condition] == METHOD_WHF) && (Error == 0) ){
		cv::Mat cluster_img_whf; // WHF
		cluster_img_whf = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		for( int i=0 ; i<ClusterIdx.size() ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			tmp(1) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			tmp(2) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			cluster_img_whf.at<cv::Vec3b>( PntIdx[SubIdx[ClusterIdx[i]]] / WIDTH, PntIdx[SubIdx[ClusterIdx[i]]] % WIDTH ) = tmp;
		}
		imgX = WHFworkPos[0];
		imgY = WHFworkPos[1];
		//cv::circle( cluster_img_whf, cv::Point(imgX, imgY), 6, cv::Scalar(0,0,255), -1, CV_AA, 0 );
		//cv::resize( cluster_img_whf, cluster_img_whf, cv::Size() , SCALE, SCALE, INTER_CUBIC);
		cv::imwrite( "cluster_img_whf.bmp", cluster_img_whf );
	}
	if( (C_FAST_SW[*itemIdx]) && (FASTError == 0) ){
		cv::Mat cluster_img_fast; // FAST
		cluster_img_fast = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );
		for( int i=0 ; i<ClusterIdx.size() ; i++ ){
			cv::Vec3b tmp;
			tmp(0) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			tmp(1) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			tmp(2) = depth[ PntIdx[SubIdx[ClusterIdx[i]]] ];
			cluster_img_fast.at<cv::Vec3b>( PntIdx[SubIdx[ClusterIdx[i]]] / WIDTH, PntIdx[SubIdx[ClusterIdx[i]]] % WIDTH ) = tmp;
		}
		//imgX = WHFworkPos[0];
		//imgY = WHFworkPos[1];
		//cv::circle( cluster_img_whf, cv::Point(imgX, imgY), 6, cv::Scalar(0,0,255), -1, CV_AA, 0 );
		//cv::resize( cluster_img_fast, cluster_img_fast, cv::Size() , SCALE, SCALE, INTER_CUBIC);
		cv::imwrite( "cluster_img_fast.bmp", cluster_img_fast );
	}

	cv::resize( masked_img, masked_img, cv::Size() , SCALE, SCALE, INTER_CUBIC);
	cv::imwrite( "sub_img.bmp", masked_img ); // �����摜�iRGB�j
	//cv::imwrite( "color_img.bmp", color_img ); // �V�[���摜�iRGB�j

	cv::Mat bg_img; // �w�i������̉摜
	bg_img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8U );
	for( int i=0 ; i<SubIdx.size() ; i++ ){
		unsigned char tmp;
		tmp = depth[ PntIdx[SubIdx[i]] ];
		bg_img.at<uchar>( PntIdx[SubIdx[i]] / WIDTH, PntIdx[SubIdx[i]] % WIDTH ) = tmp;
	}
	cv::resize( bg_img, bg_img, cv::Size() , SCALE, SCALE, INTER_CUBIC);
	cv::imwrite( "bg_img.bmp", bg_img );
	
	cv::resize( d_img, d_img, cv::Size() , SCALE, SCALE, INTER_CUBIC);
	cv::imwrite( "depth_img.bmp", d_img );
	//cv::imwrite( "depth_median_img.bmp", median_img );
#endif

#if OUTPUT2
	pcl::io::savePCDFileASCII( "cloud_subRGB.pcd", *cloud_subRGB );
	pcl::io::savePCDFileASCII( "cloud_input.pcd", *cloud );	
#endif
	fclose( fp );

	return true;

}

