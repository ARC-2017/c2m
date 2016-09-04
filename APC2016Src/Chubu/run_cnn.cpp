#include "header_cnn.h"
#include "common_params.h"
#include "loading_params.h"
#include "convolutionNetwork.h"
#include "fullConectionNetwork.h"
#include"..\\Chukyo\\common.h"
#include"..\\Chukyo\\LabelOptimization_data.h"

/* 畳み込みとプーリングのパラメータ */
class CONV_POOLING_PARAMS{
public:

	/* 畳み込みのストライド */
	int conv_stride;

	/* 畳み込みのパディングサイズ */
	int conv_padding;

	/* プーリングのストライド */
	int pooling_stride;

	/* プーリングサイズ */
	int pooling_size;

	/* バッチノーマライゼーションの有無 */
	bool batch_narmarization;

};

// 説明 CNNによる認識
// 入力： color RGB画像
// 入力： depth 距離画像
// 入力： bin_con 注目ビンの内容（中にアイテムIDが入っている）
// 入力： bin_num ビン内の物体数
// 入力： grasp 把持点群
// 出力： hyp 仮説群
bool RunCNN( cv::Mat color, cv::Mat depth, int *bin_con, int *bin_num, int nTargetItemIdx, std::vector<cv::Point>& grasp, std::vector<Hyp>& hyp, int viewpoint ){

	/* 各層の畳み込みとプーリングのパラメータ */
	CONV_POOLING_PARAMS conv_p[NUM_OF_CONV_LAYER];

	/* パラメータの初期化 */
	for(int cc = 0; cc < NUM_OF_CONV_LAYER; cc++){
		conv_p[cc].conv_stride = 0;
		conv_p[cc].conv_padding = 0;
		conv_p[cc].pooling_stride = 0;
		conv_p[cc].pooling_size = 0;
	}

	/* 1層目の畳み込みとプーリングのパラメータ */
	conv_p[0].conv_stride		= 4;
	conv_p[0].conv_padding		= 0;
	conv_p[0].pooling_stride	= 2;
	conv_p[0].pooling_size		= 3;
	conv_p[0].batch_narmarization = true;

	/* 2層目の畳み込みとプーリングのパラメータ */
	conv_p[1].conv_stride		= 1;
	conv_p[1].conv_padding		= 2;
	conv_p[1].pooling_stride	= 2;
	conv_p[1].pooling_size		= 3;
	conv_p[1].batch_narmarization = true;

	/* 3層目の畳み込みとプーリングのパラメータ */
	conv_p[2].conv_stride		= 1;
	conv_p[2].conv_padding		= 1;
	conv_p[2].pooling_stride	= 0;
	conv_p[2].pooling_size		= 0;
	conv_p[2].batch_narmarization = false;

	/* 4層目の畳み込みとプーリングのパラメータ */
	conv_p[3].conv_stride		= 1;
	conv_p[3].conv_padding		= 1;
	conv_p[3].pooling_stride	= 0;
	conv_p[3].pooling_size		= 0;
	conv_p[3].batch_narmarization = false;

	/* 5層目の畳み込みとプーリングのパラメータ */
	conv_p[4].conv_stride		= 1;
	conv_p[4].conv_padding		= 1;
	conv_p[4].pooling_stride	= 2;
	conv_p[4].pooling_size		= 3;
	conv_p[4].batch_narmarization = false;

	for(int cc = 0; cc < NUM_OF_CONV_LAYER; cc++){

		if((conv_p[cc].conv_stride == 0) &&
		   (conv_p[cc].conv_padding == 0) &&
		   (conv_p[cc].pooling_stride == 0) &&
		   (conv_p[cc].pooling_size == 0)){

			   printf("%d層目の畳み込みとプーリングのパラメータを設定してください。\n", cc + 1);
			   getchar();
			   return false;

		}

	}

	cv::Mat item_idx = cv::Mat::zeros(cv::Size(40, 1), CV_32S);

	int total_items = *bin_num;
	for(int index = 0; index < total_items; index++){
		item_idx.at<int>(0, bin_con[index]) = 1;
	}

	/* 背景を認識したくない場合，以下をコメントアウト */
	if(nTargetItemIdx != 0){
		item_idx.at<int>(0, 0) = 1;
	}

	char conv_w_path[256];
	char conv_b_path[256];

	char fc_w_path[256];
	char fc_b_path[256];

	char bn_g_path[256];
	char bn_b_path[256];
	char bn_m_path[256];
	char bn_v_path[256];

	/* 畳み込み層の重みフィルタ, バイアス, 特徴マップのクラス */
	std::vector<CONVOLUTION_LAYER> conv_layer(NUM_OF_CONV_LAYER);

	/* 全結合層の結合重み, バイアス, 応答値ユニットのクラス */
	std::vector<FULL_CONECTION_LAYER> fc_layer(NUM_OF_FC_LAYER);

	/* バッチノーマライゼーションのγ係数，β係数, 平均値, 分散値のクラス */
	std::vector<BATCH_NORMALIZATION_LAYER> bn_layer(NUM_OF_BAT_NORM);

	printf("Loading convolution parameters...\n");
	for(int conv_num = 0; conv_num < NUM_OF_CONV_LAYER; conv_num++){

		/* 畳み込み層の重みフィルタ */
		memset(conv_w_path, 0, sizeof(conv_w_path));
		sprintf(conv_w_path, "%s/conv%d_weight.npy", PARAM_DIR, conv_num + 1);

		/* 畳み込み層のバイアス */
		memset(conv_b_path, 0, sizeof(conv_b_path));
		sprintf(conv_b_path, "%s/conv%d_bias.npy", PARAM_DIR, conv_num + 1);

		/* 畳み込み層の重みフィルタとバイアスの読み込み */
		loadingConvolutionParams(conv_w_path, conv_b_path, conv_layer[conv_num].conv_w, conv_layer[conv_num].conv_b);

		/* 各層の出力特徴マップ数 */
		int map_size = conv_layer[conv_num].conv_w.size();

		/* 各層の特徴マップのサイズを設定 */
		conv_layer[conv_num].feature_map.resize(map_size);
		

	}

	printf("Loading fully conection parameters...\n");
	for(int fc_num = 0; fc_num < NUM_OF_FC_LAYER; fc_num++){

		/* 全結合層の結合重み */
		memset(fc_w_path, 0, sizeof(fc_w_path));
		sprintf(fc_w_path, "%s/l%d_weight.npy", PARAM_DIR, fc_num + 1);

		/* 全結合層のバイアス */
		memset(fc_b_path, 0, sizeof(fc_b_path));
		sprintf(fc_b_path, "%s/l%d_bias.npy", PARAM_DIR, fc_num + 1);

		loadingFullConectionParams(fc_w_path, fc_b_path, fc_layer[fc_num].fc_w, fc_layer[fc_num].fc_b);

		/* 各層の出力ユニット数 */
		int unit_size = fc_layer[fc_num].fc_w.size();

		/* 各層のユニット数を設定 */
		fc_layer[fc_num].response_unit = cv::Mat::zeros(cv::Size(unit_size, 1), CV_32F);

	}

	printf("Loading batch normalization parameters...\n");
	for(int bn_num = 0; bn_num < NUM_OF_BAT_NORM; bn_num++){

		/* バッチ正規化のγ係数 */
		memset(bn_g_path, 0, sizeof(bn_g_path));
		sprintf(bn_g_path, "%s/bn%d_gamma.npy", PARAM_DIR, bn_num + 1);

		/* バッチ正規化のβ係数 */
		memset(bn_b_path, 0, sizeof(bn_b_path));
		sprintf(bn_b_path, "%s/bn%d_beta.npy", PARAM_DIR, bn_num + 1);

		/* バッチ正規化の平均値 */
		memset(bn_m_path, 0, sizeof(bn_m_path));
		sprintf(bn_m_path, "%s/bn%d_mean.dat", PARAM_DIR, bn_num + 1);

		/* バッチ正規化の分散値 */
		memset(bn_v_path, 0, sizeof(bn_v_path));
		sprintf(bn_v_path, "%s/bn%d_var.dat", PARAM_DIR, bn_num + 1);

		loadingBatchNormalizationParams(bn_g_path, bn_b_path, bn_m_path, bn_v_path, bn_layer[bn_num].bn_g, bn_layer[bn_num].bn_b, bn_layer[bn_num].bn_mean, bn_layer[bn_num].bn_var);

	}

	printf("Loaded all parameters\n");

	/* ---畳み込み層の各サイズ------------------------------------------- */
	/*                                                                    */
	/* i層目の出力特徴マップ数       -> conv_layer[i].conv_w.size()       */
	/* i層目の入力特徴マップ数       -> conv_layer[i].conv_w[0].size()    */
	/* i層目の重みフィルタの縦サイズ -> conv_layer[i].conv_w[0][0].rows   */
	/* i層目の重みフィルタの横サイズ -> conv_layer[i].conv_w[0][0].cols   */
	/* i層目の特徴マップ数           -> conv_layer[i].feature_map.size()  */
	/* i層目の特徴マップの縦サイズ   -> conv_layer[i].feature_map[0].rows */
	/* i層目の特徴マップの横サイズ   -> conv_layer[i].feature_map[0].cols */
	/*                                                                    */
	/* ------------------------------------------------------------------ */

	/* ---全結合層の各サイズ------------------------------ */
	/*                                                     */
	/* i層目の出力ユニット数 -> fc_layer[i].fc_w.size()    */
	/* i層目の入力ユニット数 -> fc_layer[i].fc_w[0].cols   */
	/*                                                     */
	/* --------------------------------------------------- */


	/* 明るさ補正用のルックアップテーブル */
	unsigned char lut[256];

	/* 明るさの変数 1以上：明るくなる, 1未満：暗くなる */
	double Gamma = 1.0;

	if((viewpoint == 1) && (nTargetItemIdx != 0)){
		Gamma = 1.0;
	}
	else if((viewpoint == 1) && (nTargetItemIdx == 0)){
		Gamma = 0.8;
	}
	else if(viewpoint == 2){
		Gamma = 1.2;
	}

	/* ルックアップテーブルの計算 */
	double gm = 1.0 / Gamma;
	for(int n = 0; n < 256; n++){
		lut[n] = pow(1.0 * (double)n / 255.0, gm) * 255.0;
	}


	/* 把持点周辺領域を切り出したカラーパッチ画像 */
	cv::Mat color_patch;

	/* 把持点周辺領域を切り出したパッチ画像のリサイズ画像 */
	cv::Mat resize_patch;

	/* 明るさ補正したパッチ画像 */
	cv::Mat ill_patch;

	/* 把持点の認識ラベル */
	std::vector<int> recg_label(grasp.size());

	/* 把持地の認識スコア */
	std::vector<float> recg_score(grasp.size());

	/* RGB画像配列 */
	std::vector<cv::Mat> rgb(3);

	/* RGBD画像の初期化 */
	for(int i = 0; i < 3; i++){
		rgb[i] = cv::Mat::zeros(cv::Size(CNN_PATCH_SIZE, CNN_PATCH_SIZE), CV_32F);
	}

	/* 把持点の数だけループ */
	for(int gp = 0; gp < grasp.size(); gp++){

		printf("------------Grasping point : %d\n", gp + 1);

		/* カラー画像の把持点周辺領域を切り出し */
		cv::getRectSubPix(color, cv::Size(PATCH_SIZE, PATCH_SIZE), grasp[gp], color_patch);

		/* 画像をアップサンプリング */
		cv::resize(color_patch, resize_patch, cv::Size(CNN_PATCH_SIZE, CNN_PATCH_SIZE), 0, 0, cv::INTER_CUBIC);

		/* 画像の明るさ補正 */
		cv::LUT(resize_patch, cv::Mat(cv::Size(256, 1), CV_8U, lut), ill_patch);

		/* RGB画像の生成 */
		for(int y = 0; y < resize_patch.rows; y++){
			for(int x = 0; x < resize_patch.cols; x++){
				rgb[0].at<float>(y, x) = (float)ill_patch.data[resize_patch.step * y + resize_patch.channels() * x + 0] / 255.0;
				rgb[1].at<float>(y, x) = (float)ill_patch.data[resize_patch.step * y + resize_patch.channels() * x + 1] / 255.0;
				rgb[2].at<float>(y, x) = (float)ill_patch.data[resize_patch.step * y + resize_patch.channels() * x + 2] / 255.0;
			}
		}




		bool convolution_res, fullconection_res, activation_res, pooling_res, batch_res;


		/* 畳み込み層の処理 */
		for(int conv_num = 0; conv_num < NUM_OF_CONV_LAYER; conv_num++){

			printf("畳み込み処理%d層目\n", conv_num + 1);

			/* 1層目は入力画像を畳み込み層に入力, 2層目以降は1つ前の層の特徴マップを入力 */
			if(conv_num == 0){	
				/* 畳み込み処理 */
				convolution_res = convolutionLayer(rgb, conv_layer[conv_num].feature_map, conv_layer[conv_num].conv_w, conv_layer[conv_num].conv_b, conv_p[conv_num].conv_stride, conv_p[conv_num].conv_padding);
				/* バッチ正規化 */
				if(conv_p[conv_num].batch_narmarization){
					batch_res = batchNormalization(conv_layer[conv_num].feature_map, bn_layer[conv_num].bn_g, bn_layer[conv_num].bn_b, bn_layer[conv_num].bn_mean, bn_layer[conv_num].bn_var);
				}
				/* 活性化関数 */
				activation_res = convReLU(conv_layer[conv_num].feature_map);
				/* プーリング */
				if(conv_p[conv_num].pooling_size != 0){
					pooling_res = maxPooling(conv_layer[conv_num].feature_map, conv_p[conv_num].pooling_stride, conv_p[conv_num].pooling_size);
				}

				if(!convolution_res){
					printf("畳み込み失敗。\n");
					return false;
				}
			}
			else{
				/* 畳み込み処理 */
				convolution_res = convolutionLayer(conv_layer[conv_num - 1].feature_map, conv_layer[conv_num].feature_map, conv_layer[conv_num].conv_w, conv_layer[conv_num].conv_b, conv_p[conv_num].conv_stride, conv_p[conv_num].conv_padding);
				/* バッチ正規化 */
				if(conv_p[conv_num].batch_narmarization){
					batch_res = batchNormalization(conv_layer[conv_num].feature_map, bn_layer[conv_num].bn_g, bn_layer[conv_num].bn_b, bn_layer[conv_num].bn_mean, bn_layer[conv_num].bn_var);
				}
				/* 活性化関数 */
				activation_res = convReLU(conv_layer[conv_num].feature_map);
				/* プーリング */
				if(conv_p[conv_num].pooling_size != 0){
					pooling_res = maxPooling(conv_layer[conv_num].feature_map, conv_p[conv_num].pooling_stride, conv_p[conv_num].pooling_size);
				}

				if(!convolution_res){
					printf("畳み込み失敗。\n");
					return false;
				}
			}

		}

		int conv_output_size = conv_layer[NUM_OF_CONV_LAYER - 1].feature_map.size() * conv_layer[NUM_OF_CONV_LAYER - 1].feature_map[0].rows * conv_layer[NUM_OF_CONV_LAYER - 1].feature_map[0].cols;

		/* 畳み込み最終層の配列 */
		cv::Mat conv_output = cv::Mat::zeros(cv::Size(conv_output_size, 1), CV_32F);

		/* 畳み込み最終層の全特徴マップを1次元に変換 */
		size_t conv_out_num = 0;
		for(int ch = 0; ch < conv_layer[NUM_OF_CONV_LAYER - 1].feature_map.size(); ch++){
			for(int row = 0; row < conv_layer[NUM_OF_CONV_LAYER - 1].feature_map[0].rows; row++){
				for(int col = 0; col < conv_layer[NUM_OF_CONV_LAYER - 1].feature_map[0].cols; col++){

					conv_output.at<float>(0, conv_out_num) =  conv_layer[NUM_OF_CONV_LAYER - 1].feature_map[ch].at<float>(row, col);
					conv_out_num++;

				}
			}
		}

		if(fc_layer[0].fc_w[0].cols != conv_output.cols){
			printf("畳み込みの出力次元数(%d)と全結合の入力次元数(%d)が一致しません．\n", conv_output.cols, fc_layer[0].fc_w[0].cols);
			return false;
		}


		/* クラス確率 */
		cv::Mat probability;

		/* 全結合層の処理 */
		for(int fc_num = 0; fc_num < NUM_OF_FC_LAYER; fc_num++){

			printf("全結合処理%d層目\n", fc_num + 1);

			/* 1層目は畳み込み層の出力を全結合層に入力, 2層目意向は1つ前の層の応答値ユニットを入力, 最終層の活性化関数はSoftmax */
			if(fc_num == 0){
				/* 全結合処理 */
				fullconection_res = fullConectionLayer(conv_output, fc_layer[fc_num].response_unit, fc_layer[fc_num].fc_w, fc_layer[fc_num].fc_b);
				/* 活性化関数 */
				activation_res = fcReLU(fc_layer[fc_num].response_unit);
			}
			else if(fc_num == NUM_OF_FC_LAYER - 1){
				/* 全結合処理 */
				fullconection_res = fullConectionLayer(fc_layer[fc_num - 1].response_unit, fc_layer[fc_num].response_unit, fc_layer[fc_num].fc_w, fc_layer[fc_num].fc_b);
				/* 通常のSoftMax関数 */
				//activation_res = softMax(fc_layer[fc_num].response_unit, probability);
				/* 制約付きSoftMax関数 */
				activation_res = conditionalSoftMax(fc_layer[fc_num].response_unit, probability, item_idx);
			}
			else{
				/* 全結合処理 */
				fullconection_res = fullConectionLayer(fc_layer[fc_num - 1].response_unit, fc_layer[fc_num].response_unit, fc_layer[fc_num].fc_w, fc_layer[fc_num].fc_b);
				/* 活性化関数 */
				activation_res = fcReLU(fc_layer[fc_num].response_unit);
			}


		}

		cv::Mat sorted_score;
		cv::Mat sorted_index;

		cv::sort(probability, sorted_score, CV_SORT_EVERY_ROW|CV_SORT_DESCENDING);
		cv::sortIdx(probability, sorted_index, CV_SORT_EVERY_ROW|CV_SORT_DESCENDING);

		recg_label[gp] = sorted_index.at<int>(0, 0);
		recg_score[gp] = sorted_score.at<float>(0, 0);

		Hyp tmp_hyp;
		tmp_hyp.grasp.x = (int)grasp[gp].x;
		tmp_hyp.grasp.y = (int)grasp[gp].y;
		tmp_hyp.label = recg_label[gp];
		tmp_hyp.score = ( double )recg_score[gp];
		tmp_hyp.method = CNN_LABEL;
		hyp.push_back( tmp_hyp );

	}


	/* -----↓↓以下vector配列のクリア------------------------------------------------ */

	rgb.clear();

	std::vector<cv::Mat> ().swap(rgb);

	recg_label.clear();
	recg_score.clear();

	std::vector<int> ().swap(recg_label);
	std::vector<float> ().swap(recg_score);

	for(int conv_num = 0; conv_num < NUM_OF_CONV_LAYER; conv_num++){

		conv_layer[conv_num].conv_w.clear();
		conv_layer[conv_num].conv_b.clear();
		conv_layer[conv_num].feature_map.clear();

		std::vector< std::vector<cv::Mat> > ().swap(conv_layer[conv_num].conv_w);
		std::vector<float> ().swap(conv_layer[conv_num].conv_b);
		std::vector<cv::Mat> ().swap(conv_layer[conv_num].feature_map);

	}

	for(int fc_num = 0; fc_num < NUM_OF_FC_LAYER; fc_num++){

		fc_layer[fc_num].fc_w.clear();
		fc_layer[fc_num].fc_b.clear();

		std::vector<cv::Mat> ().swap(fc_layer[fc_num].fc_w);
		std::vector<float> ().swap(fc_layer[fc_num].fc_b);

	}

	for(int bn_num = 0; bn_num < NUM_OF_BAT_NORM; bn_num++){

		bn_layer[bn_num].bn_g.clear();
		bn_layer[bn_num].bn_b.clear();
		bn_layer[bn_num].bn_mean.clear();
		bn_layer[bn_num].bn_var.clear();

		std::vector<float> ().swap(bn_layer[bn_num].bn_g);
		std::vector<float> ().swap(bn_layer[bn_num].bn_b);
		std::vector<float> ().swap(bn_layer[bn_num].bn_mean);
		std::vector<float> ().swap(bn_layer[bn_num].bn_var);

	}

	conv_layer.clear();
	fc_layer.clear();
	bn_layer.clear();

	std::vector<CONVOLUTION_LAYER> ().swap(conv_layer);
	std::vector<FULL_CONECTION_LAYER> ().swap(fc_layer);
	std::vector<BATCH_NORMALIZATION_LAYER> ().swap(bn_layer);


	return true;

}

//cv::Mat visualizationMap(cv::Mat map2d){
//
//	cv::Mat visualization = cv::Mat::zeros(cv::Size(map2d.cols, map2d.rows), CV_8UC1);
//
//	cv::Mat max, min;
//
//	cv::reduce(map2d, max, 0, CV_REDUCE_MAX);
//	cv::reduce(max, max, 1, CV_REDUCE_MAX);
//
//	cv::reduce(map2d, min, 0, CV_REDUCE_MIN);
//	cv::reduce(min, min, 1, CV_REDUCE_MIN);
//
//	int dmax = 255;
//	int dmin = 0;
//
//	for(int j = 0; j < map2d.rows; j++){
//		for(int i = 0; i < map2d.cols; i++){
//
//			visualization.at<unsigned char>(j, i) = (unsigned char)((float)(dmax - dmin) * (map2d.at<float>(j, i) - min.at<float>(0, 0)) / (max.at<float>(0, 0) - min.at<float>(0, 0)));
//
//		}
//	}
//
//
//
//	return visualization;
//}
