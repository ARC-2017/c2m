/*Hough Forests*/ 
// target APC
// Ryuei Murata 
// mryua@vision.cs.chubu.ac.jp
// �p�����[�^��WHFcommon.h�ɂĒ�`���Ă��܂��B
// �ŏI�X�V�� 2015.4.8


#include "WHFcommon.h"
#include "Detector.h"



bool RecgWeightedHoughForest(int *itemIdx, cv::Mat color, double *workPos, double *score,int count );

int main(){
	//�ȈՓI�Ƀ|�C���^�ϐ��̃A�h���X��bin�ԍ��ƃA�C�e���ԍ�,���͉摜��^���Ă��܂��B
	////////////////////////////////////////////////////////////////////////////////////

	char img_name[256];
	int BINNUM=1,ITEMIDX=13;
	double SCORE=0.0;
	int *itemIdx;
	itemIdx = &ITEMIDX;
	double workPos[2]={0.0};
	double *score;
	score =&SCORE;	
	

	for(int s=1;s<2;s++){
		sprintf(img_name,"./color.bmp");//image_pass
		cv::Mat color = cv::imread(img_name,1);

		if( !color.data ){
			cout << "\ncan not read image  " <<img_name<< endl;
		return false;
		}
	////////////////////////////////////////////////////////////////////////////////////	


	//RecgWeightedHoughForest
	RecgWeightedHoughForest(itemIdx,color,workPos,score,s);

	//std::cout<<"���[�N�ʒu�F"<<workPos[0]<<","<<workPos[1]<<"�@�X�R�A�F"<<*score<<std::endl;
	}
}

//
bool RecgWeightedHoughForest(int *itemIdx, cv::Mat color, double *workPos,double *score ,int count){

		int bin,itemNum;
		cv::Mat testImage;
		//1�`�����l���̃O���[�X�X�P�[���摜���쐬
		cv::cvtColor(color,testImage,CV_BGR2GRAY,1);
		itemNum=*itemIdx;

		Detector detector;
		//���͉摜�̃X�P�[����WHFcommon.h�Œ�`����SCALE�̒l�ɕϊ�(�`�������W�����̃X�P�[���ɂ��킹�Ē���)
		resize(testImage, testImage, Size() , SCALE, SCALE, INTER_CUBIC);
		//���o�p�֐�
		detector.computeError(itemNum,testImage,workPos,score,count);

		//ROG�����o���̔���
		#ifdef LOGMODE
		std::ofstream ofs( "./WHF_LOG.txt" ,std::ios::app);
		ofs<<"\titemIdx="<<itemNum<<"\tworkPos="<<workPos[0]<<","<<workPos[1]<<"\tscore="<<*score<<std::endl;
		ofs.close();
		#endif

		return true;
}