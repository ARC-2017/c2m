
#include "header_cnn.h"
#include "convolutionNetwork.h"


bool convolutionLayer(std::vector<cv::Mat> input_map, std::vector<cv::Mat> &output_map, std::vector< std::vector<cv::Mat> > conv_w, std::vector<float> bias, int stride, int pad){

	/* 重みフィルタの半径 */
	int filter_r = (int)floor((float)conv_w[0][0].cols * 0.5);
	int pad_diff = filter_r - pad;

	if (pad_diff < 0){
		printf("パディングサイズを%d以下にしてください。\n", filter_r);
		return false;
	}
	if (stride < 0){
		printf("ストライドを1以上にしてください。\n");
		return false;
	}

	/* 出力チャンネル数 */
	int output_channel = conv_w.size();
	/* 入力チャンネル数 */
	int input_channel = conv_w[0].size();
	/* 重みフィルタサイズ */
	int filter_size = conv_w[0][0].cols;


	/* 出力特徴マップの幅 */
	int output_map_width = input_map[0].cols - (pad_diff * 2);
	/* 出力特徴マップの高さ */
	int output_map_height = input_map[0].rows - (pad_diff * 2);

	/* ストライドを考慮して出力特徴マップのサイズを計算 */
	if (stride > 1){
		output_map_width = (int)ceil((float)output_map_width / (float)stride);
		output_map_height = (int)ceil((float)output_map_height / (float)stride);
	}

	cv::Mat conv_val;
	cv::Mat omap;


	/* 出力特徴マップのループ */
	for (int och = 0; och < output_channel; och++){

		/* 出力特徴マップの初期化 */
		omap = cv::Mat::zeros(cv::Size(output_map_width, output_map_height), CV_32F);

		/* 入力特徴マップのループ */
		for (int ich = 0; ich < input_channel; ich++){

			conv_val;

			/* 畳み込み処理 */
			cv::filter2D(input_map[ich], conv_val, -1, conv_w[och][ich], cv::Point(-1, -1), 0.0, IPL_BORDER_CONSTANT);

			if ((pad == filter_r) && (stride == 1)){
				/* パディングとストライド処理が不要な場合は畳み込み結果をそのまま特徴マップに加算 */
				omap += conv_val.clone();
			}
			else{
				/* パディングとストライド処理 */
				for (int y = pad_diff, oy = 0; y < (conv_val.rows - pad_diff); y += stride, oy++){
					for (int x = pad_diff, ox = 0; x < (conv_val.cols - pad_diff); x += stride, ox++){
						omap.at<float>(oy, ox) += conv_val.at<float>(y, x);
					}
				}
			}

		}

		/* 出力特徴マップのバイアスを加算 */
		output_map[och] = omap + bias[och];

	}

	return true;
}


/* Batch Normalization関数 */
bool batchNormalization(std::vector<cv::Mat> &feature_map, std::vector<float> gamma, std::vector<float> beta, std::vector<float> mean, std::vector<float> var){

	int map_size = feature_map.size();
	float exp = 1e-5;
	cv::Mat tmp_map;

	for(int i = 0; i < map_size; i++){

		tmp_map = feature_map[i].clone();

		tmp_map = tmp_map - mean[i];
		tmp_map = tmp_map / sqrt(var[i] + exp);

		feature_map[i] = tmp_map * gamma[i] + beta[i];

	}

	return true;
}

/* ReLU関数 */
bool convReLU(std::vector<cv::Mat> &feature_map){

	for(int i = 0; i < feature_map.size(); i++){
		feature_map[i] = cv::max(0, feature_map[i]);
	}

	return true;
}


bool maxPooling(std::vector<cv::Mat> &feature_map, int pooling_stride, int pooling_size){

	int pooling_r = (int)floor((float)pooling_size * 0.5);

	int down_size = (int)floor((float)feature_map[0].cols / (float)pooling_stride);
	int map_width = feature_map[0].cols;
	int map_height = feature_map[0].rows;

	for(int i = 0; i < feature_map.size(); i++){
		
		cv::Mat pooling_map = cv::Mat::zeros(cv::Size(down_size, down_size), CV_32F);
		cv::Mat feat_map = feature_map[i].clone();

		for(int r = pooling_r, rp = 0; r < (map_height - pooling_r); r += pooling_stride, rp++){
			for(int c = pooling_r, cp = 0; c < (map_width - pooling_r); c += pooling_stride, cp++){

				float max_val = -FLT_MAX;

				for(int poly = -pooling_r; poly <= pooling_r; poly++){
					for(int polx = -pooling_r; polx <= pooling_r; polx++){

						int xi = c + polx;
						int yi = r + poly;

						if((xi < feat_map.cols) && (yi < feat_map.rows)){
							if(feat_map.at<float>(yi, xi) > max_val)
								max_val = feat_map.at<float>(yi, xi);
						}

					}
				}

				pooling_map.at<float>(rp, cp) = max_val;

			}
		}

		feature_map[i] = pooling_map.clone();

	}


	return true;
}