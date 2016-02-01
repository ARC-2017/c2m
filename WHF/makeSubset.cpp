#include "makeSubset.h"

using namespace std;
DOT_IH DOT;
bool feature[FEATURE];

//�e�X�g�摜����p�b�`��؂�o��
vector<testSample> makeSubset::extractPatch( Mat &testImage ){
	vector<testSample> testPaches;
	testSample sampletmp;
	dot_feature Testdot[CELL_NUM];
	int patchNum;
	int im_clos;
	int im_rows;
	//cout << "�e�X�g�p�p�b�`���o��....\r";
		
		im_clos = testImage.cols;
		im_rows = testImage.rows;

		patchNum = 0;

		DOT.SetupFeature();		//������
		DOT.CreateIntegralHistogram(testImage);	//Integral Histogram�̍쐬

		cv::Mat clone;
		//�e�X�g�摜����p�b�`��؂�o��DOT�����𒊏o
		for( int y = 0; ( y + PATCH_HEIGHT ) <= im_rows; y += TEST_SS ){
			for( int x = 0; ( x + PATCH_WIDTH ) <= im_clos; x += TEST_SS ){

				//if(testImage.at<int>(x + PATCH_WIDTH / 2 , y + PATCH_HEIGHT / 2)!=0){
				//if(testImage.at<int>(x , y)!=0){
				//if(testImage.data[y*testImage.step+x]!=0){
				Mat tmpPatch( testImage, Rect( x, y, PATCH_WIDTH, PATCH_HEIGHT ) );


				//DOT�����̒��o
				DOT.Getfeature(Testdot, x, y, PATCH_WIDTH, PATCH_HEIGHT);	//�����ʒ��o

				//testSample�Ɋi�[
				sampletmp.insert( tmpPatch, Point( x + PATCH_WIDTH / 2 , y + PATCH_HEIGHT / 2 ), Testdot );
				
				//�p�b�`�f�[�^��testSample�z��Ɋi�[
				testPaches.push_back( sampletmp );

				//printf("�p�b�`��:%d\r", patchNum);
				patchNum++;
				//}	//rateIFend
				//}
			}
		}
		DOT.SizeInit();			//�̈�J��
		sampletmp;

	return testPaches;
}


//
makeSubset::makeSubset(void)
{
}


makeSubset::~makeSubset(void)
{
}
