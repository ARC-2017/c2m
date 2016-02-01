#define _CRT_SECURE_NO_WARNINGS
#include "sample.h"

DOT_IH::DOT_IH(void)
{
}

DOT_IH::~DOT_IH(void)
{
}

//�����ʂ̏����i�[
int DOT_IH::SetupFeature(){
	int cell_w, cell_h;
	int cell = 0;
	int feature = 0;

	cell_w = CELL_X;
	cell_h = CELL_Y;

	//���������̃u���b�N�̈ړ�
	for(int q = 0; q < cell_h; q++){
		//x�������̃u���b�N�̈ړ�
		for(int p = 0;  p < cell_w; p++){

					//�q�X�g�O�����̗ʎq���ԍ��̈ړ�
					for(int bin = 0; bin < ORIENTATION; bin++){

						//�ʎq���ԍ��̊i�[
						setDOT[feature].bin = bin;

						feature++;
					}
			cell++;
		}
	}

	//�u���b�N�̑������i�[
	sum_block = cell;

	//���z�����C���z���x��LUT���쐬
	CreateGradLUT();

	feature_num = FEATURE;

	for(int i = 0; i < FEATURE; i++){

		//BIT_NUMbit���Ɋi�[
		BinaryLUT[i] = (int)(i / ORIENTATION);
	}

	return 0;
}

//���z�����C���z���x��LUT���쐬
int DOT_IH::CreateGradLUT(){

	double x, y;
	double grad;

	// ���z�����̎Z�o
	for(int j = 0; j < LUT_SIZE; j++){

		// 0����511��-255����255�ɕϊ�
		y = j - 255.0;

		for(int i = 0; i < LUT_SIZE; i++){

			// 0�`511��-255�`255�ɕϊ�
			x = i - 255.0;

			//���z���x�̎Z�o
			magnitudeLUT[j][i] = sqrt(x * x + y * y);

			//���z�����̎Z�o
			grad = atan2(y, x);

			double grad1 = grad;
			//printf("���z�����F%lf\n", grad);

			//���W�A������p�x�֕ϊ�
			grad = (grad * 180) / CV_PI;

//			grad - 5.0;

			//�}�C�i�X�̊p�x�͔��]������
			if(grad < 0.0){

				grad += 360.0;
			}

			//0�`360�x��0�`180�x�ɂ���
			if( ANGLE < grad ){

				grad -= 180.0;
			}

//			grad = (int)grad % 360;

			//�p�x�𕪊�
			if( (int)( floor( grad / (ANGLE / (double)ORIENTATION) ) ) == 8 ){

				gradLUT[j][i] = (int)( floor( grad / (ANGLE / (double)ORIENTATION) ) ) - 1;
			}else{

				gradLUT[j][i] = (int)( floor( grad / (ANGLE / (double)ORIENTATION) ) );
			}
			if( (int)(grad / (ANGLE / (double)ORIENTATION)) == 8 ){

			}
		}
	}
	
	return 0;
}

//�C���e�O�����q�X�g�O�����̍쐬
int DOT_IH::CreateIntegralHistogram(const cv::Mat &img){	

	int xgrad, ygrad;
	int grad;
	int wStep;
	int img_h;
	unsigned char* imgSource;
	cv::Mat check = cv::Mat::zeros(cv::Size(PATCH_WIDTH, PATCH_HEIGHT), CV_8UC1);

	img_h = int(img.rows);
	img_w = int(img.cols);
	wStep = int(img.step);
	wiStep = int(img.step);

	std::size_t cols = img.cols;
	std::size_t rows = img.rows;

	imgSource = (unsigned char*) img.data;
	imgSource2 = (unsigned char*) img.data;

	cv::Mat Grad = cv::Mat::zeros(cv::Size(img.cols, img.rows), CV_8UC1);

	//�̈�m��
	for(int k = 0; k < ORIENTATION; k++){
	
		image[k] = new double* [cols];

		for(int i = 0; i < cols; i++){
			image[k][i] = new double [rows];
			//������
			for(int j = 0; j < img.rows; j++){
				image[k][i][j] = 0.0;
			}
		}

	}

	//���������̈ړ�
	for(int y = 0; y < rows; y++){
		//x�������̈ړ�
		for(int x = 0; x < cols; x++){
			//����
			if(x == 0){
				xgrad = imgSource[y * wStep + (x + 0)] - imgSource[y * wStep + (x + 1)];
			}
			else if(x == (cols-1)){
				xgrad = imgSource[y * wStep + (x - 1)] - imgSource[y * wStep + (x + 0)];
			}
			else{
				xgrad = imgSource[y * wStep + (x - 1)] - imgSource[y * wStep + (x + 1)];						
			}
			if(y == 0){
				ygrad = imgSource[(y + 0) * wStep + x] - imgSource[(y + 1) * wStep + x];
			}
			else if(y == (rows-1)){	
				ygrad = imgSource[(y - 1) * wStep + x] - imgSource[(y + 0) * wStep + x];
			}
			else{	
				ygrad = imgSource[(y - 1) * wStep + x] - imgSource[(y + 1) * wStep + x];
			}

			xgrad = xgrad + 255;
			ygrad = ygrad + 255;

			//���z����
			grad = gradLUT[ygrad][xgrad];

			//���z���x
			image[grad][x][y] = magnitudeLUT[ygrad][xgrad];
		}
	}

	//�e������Integral Image�̎Z�o
	CompIntegralImage(img.cols, img.rows);

	return 0;

}

//�e�����̃C���e�O�����C���[�W�̎Z�o
int DOT_IH::CompIntegralImage(int width, int height){

	for(int k = 0; k < ORIENTATION; k++){
		//y�������̉��Z
		for(int j = 1; j < height; j++){
			for(int i = 0; i < width; i++){
				image[k][i][j] = image[k][i][j-1] + image[k][i][j];
			}
		}

		//x�������̉��Z
		for(int j = 0; j < height; j++){
			for(int i = 1; i < width; i++){
				image[k][i][j] = image[k][i-1][j] + image[k][i][j];
			}
		}
	}

	return 0;

}

//�w��͈͓��̃C���e�O�����C���[�W���Z�o
int DOT_IH::Getfeature(dot_feature* dot, int x, int y, int w_size, int h_size){
	int x1, x2, y1, y2;
	int iw, ih;
	int Cgrad;
	int Cxgrad, Cygrad;
	double Cmag;

	//1�Z�������s�N�Z�����Z�o
	iw = w_size / CELL_X;
	ih = h_size / CELL_Y;

	for(int q = 0; q < CELL_Y; q++){
		for(int p = 0; p < CELL_X; p++){
			for(int k = 0; k < ORIENTATION; k++ ){

				cell_center[p][q][k] = 0.0;
			}
		}
	}

	//�̈���̌��z�����q�X�g�O�����̍쐬
	for(int q = 0; q < CELL_Y; q++){
		y1 = y  + ih * q;
		y2 = y1 + ih - 1;
		for(int p = 0; p < CELL_X; p++){
			x1 = x  + iw * p;
			x2 = x1 + iw - 1;

			int X = (int)(( x1 + x2 ) / 2.0);
			int Y = (int)(( y1 + y2 ) / 2.0);

			Cxgrad = imgSource2[ Y * wiStep + ( X - 1 ) ] - imgSource2[ Y * wiStep + ( X + 1 ) ];
			Cygrad = imgSource2[ ( Y - 1 ) * wiStep + X ] - imgSource2[ ( Y + 1 ) * wiStep + X ];

			Cxgrad = Cxgrad + 255;
			Cygrad = Cygrad + 255;

			Cgrad = gradLUT[ Cygrad ][ Cxgrad ];
			Cmag = magnitudeLUT[ Cygrad ][ Cxgrad ];

			//�Z�����̒��ډ�f�̌��z���
			cell_center[p][q][Cgrad] = Cmag;

			if(x1 == 0){
				if(y1 == 0){
					for(int k = 0; k < ORIENTATION; k++){
						cell_hist[p][q][k] = image[k][x2][y2];
					}
				}
				else{
					for(int k = 0; k < ORIENTATION; k++){		
						cell_hist[p][q][k] = image[k][x2][y2] - image[k][x2][y1-1];
					}
				}
			}
			else{
				if(y1==0){
					for(int k = 0; k < ORIENTATION; k++){
						cell_hist[p][q][k] = image[k][x2][y2] - image[k][x1-1][y2];
					}
				}else{
					for(int k = 0; k < ORIENTATION; k++){
						cell_hist[p][q][k] = (image[k][x1-1][y1-1] + image[k][x2][y2]) - (image[k][x1-1][y2] + image[k][x2][y1-1]);
					}
				}
			}
		}	
	}

	//�q�X�g�O�����̐��K���Ɠ����ʂ̎擾
	Normalize( dot );

	return 0;

}

//DOT�̎擾
int DOT_IH::Normalize( dot_feature* dot ){
	int box_w, box_h;
	double sum_magnitude;
	double sum_center_magnitude;
	int num = 0;
	int cell_id = 0;
	unsigned int tmp0_180;
	unsigned int tmp180_360;
	unsigned int tmp_center;

	box_w = CELL_X;
	box_h = CELL_Y;

	//���������̃u���b�N�̈ړ�
	for(int q = 0; q < box_h; q++){
		//x�������̃u���b�N�̈ړ�
		for(int p = 0; p < box_w; p++){
			sum_magnitude = 0.0;
			sum_center_magnitude = 0.0;
			tmp0_180 = 0;
			tmp180_360 = 0;
			tmp_center = 0;

			//�q�X�g�O�����̗ʎq���ԍ��̈ړ�
			for(int bin = 0; bin < ORIENTATION; bin++){
				if(BIT_TH <= cell_hist[p][q][bin]){
					tmp0_180 += B_LUT[bin];
				}
			}
			dot[cell_id].feature0_180 = tmp0_180;

			cell_id++;
		}
	}

	return 0;

}

//�̈�J��
int DOT_IH::SizeInit(){

	//�C���e�O�����q�X�g�O�����̗̈�J��
	for(int k = 0; k < ORIENTATION; k++){
		for(int i = 0; i < img_w; i++){

			delete [] image[k][i];
		}

		delete [] image[k];
	}

	return 0;
}








