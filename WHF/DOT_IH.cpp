#define _CRT_SECURE_NO_WARNINGS
#include "sample.h"

DOT_IH::DOT_IH(void)
{
}

DOT_IH::~DOT_IH(void)
{
}

//特徴量の情報を格納
int DOT_IH::SetupFeature(){
	int cell_w, cell_h;
	int cell = 0;
	int feature = 0;

	cell_w = CELL_X;
	cell_h = CELL_Y;

	//ｙ軸方向のブロックの移動
	for(int q = 0; q < cell_h; q++){
		//x軸方向のブロックの移動
		for(int p = 0;  p < cell_w; p++){

					//ヒストグラムの量子化番号の移動
					for(int bin = 0; bin < ORIENTATION; bin++){

						//量子化番号の格納
						setDOT[feature].bin = bin;

						feature++;
					}
			cell++;
		}
	}

	//ブロックの総数を格納
	sum_block = cell;

	//勾配方向，勾配強度のLUTを作成
	CreateGradLUT();

	feature_num = FEATURE;

	for(int i = 0; i < FEATURE; i++){

		//BIT_NUMbit毎に格納
		BinaryLUT[i] = (int)(i / ORIENTATION);
	}

	return 0;
}

//勾配方向，勾配強度のLUTを作成
int DOT_IH::CreateGradLUT(){

	double x, y;
	double grad;

	// 勾配方向の算出
	for(int j = 0; j < LUT_SIZE; j++){

		// 0から511を-255から255に変換
		y = j - 255.0;

		for(int i = 0; i < LUT_SIZE; i++){

			// 0～511を-255～255に変換
			x = i - 255.0;

			//勾配強度の算出
			magnitudeLUT[j][i] = sqrt(x * x + y * y);

			//勾配方向の算出
			grad = atan2(y, x);

			double grad1 = grad;
			//printf("勾配方向：%lf\n", grad);

			//ラジアンから角度へ変換
			grad = (grad * 180) / CV_PI;

//			grad - 5.0;

			//マイナスの角度は反転させる
			if(grad < 0.0){

				grad += 360.0;
			}

			//0～360度を0～180度にする
			if( ANGLE < grad ){

				grad -= 180.0;
			}

//			grad = (int)grad % 360;

			//角度を分割
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

//インテグラルヒストグラムの作成
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

	//領域確保
	for(int k = 0; k < ORIENTATION; k++){
	
		image[k] = new double* [cols];

		for(int i = 0; i < cols; i++){
			image[k][i] = new double [rows];
			//初期化
			for(int j = 0; j < img.rows; j++){
				image[k][i][j] = 0.0;
			}
		}

	}

	//ｙ軸方向の移動
	for(int y = 0; y < rows; y++){
		//x軸方向の移動
		for(int x = 0; x < cols; x++){
			//差分
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

			//勾配方向
			grad = gradLUT[ygrad][xgrad];

			//勾配強度
			image[grad][x][y] = magnitudeLUT[ygrad][xgrad];
		}
	}

	//各方向のIntegral Imageの算出
	CompIntegralImage(img.cols, img.rows);

	return 0;

}

//各方向のインテグラルイメージの算出
int DOT_IH::CompIntegralImage(int width, int height){

	for(int k = 0; k < ORIENTATION; k++){
		//y軸方向の加算
		for(int j = 1; j < height; j++){
			for(int i = 0; i < width; i++){
				image[k][i][j] = image[k][i][j-1] + image[k][i][j];
			}
		}

		//x軸方向の加算
		for(int j = 0; j < height; j++){
			for(int i = 1; i < width; i++){
				image[k][i][j] = image[k][i-1][j] + image[k][i][j];
			}
		}
	}

	return 0;

}

//指定範囲内のインテグラルイメージを算出
int DOT_IH::Getfeature(dot_feature* dot, int x, int y, int w_size, int h_size){
	int x1, x2, y1, y2;
	int iw, ih;
	int Cgrad;
	int Cxgrad, Cygrad;
	double Cmag;

	//1セルが何ピクセルか算出
	iw = w_size / CELL_X;
	ih = h_size / CELL_Y;

	for(int q = 0; q < CELL_Y; q++){
		for(int p = 0; p < CELL_X; p++){
			for(int k = 0; k < ORIENTATION; k++ ){

				cell_center[p][q][k] = 0.0;
			}
		}
	}

	//領域内の勾配方向ヒストグラムの作成
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

			//セル内の注目画素の勾配情報
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

	//ヒストグラムの正規化と特徴量の取得
	Normalize( dot );

	return 0;

}

//DOTの取得
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

	//ｙ軸方向のブロックの移動
	for(int q = 0; q < box_h; q++){
		//x軸方向のブロックの移動
		for(int p = 0; p < box_w; p++){
			sum_magnitude = 0.0;
			sum_center_magnitude = 0.0;
			tmp0_180 = 0;
			tmp180_360 = 0;
			tmp_center = 0;

			//ヒストグラムの量子化番号の移動
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

//領域開放
int DOT_IH::SizeInit(){

	//インテグラルヒストグラムの領域開放
	for(int k = 0; k < ORIENTATION; k++){
		for(int i = 0; i < img_w; i++){

			delete [] image[k][i];
		}

		delete [] image[k];
	}

	return 0;
}








