#include "header_cnn.h"
#include "common_params.h"
#include "loading_params.h"
#include "convolutionNetwork.h"
#include "fullConectionNetwork.h"
#include"..\\Chukyo\\common.h"
#include"..\\Chukyo\\LabelOptimization_data.h"

/* ��ݍ��݂ƃv�[�����O�̃p�����[�^ */
class CONV_POOLING_PARAMS{
public:

	/* ��ݍ��݂̃X�g���C�h */
	int conv_stride;

	/* ��ݍ��݂̃p�f�B���O�T�C�Y */
	int conv_padding;

	/* �v�[�����O�̃X�g���C�h */
	int pooling_stride;

	/* �v�[�����O�T�C�Y */
	int pooling_size;

	/* �o�b�`�m�[�}���C�[�[�V�����̗L�� */
	bool batch_narmarization;

};

// ���� CNN�ɂ��F��
// ���́F color RGB�摜
// ���́F depth �����摜
// ���́F bin_con ���ڃr���̓��e�i���ɃA�C�e��ID�������Ă���j
// ���́F bin_num �r�����̕��̐�
// ���́F grasp �c���_�Q
// �o�́F hyp �����Q
bool RunCNN( cv::Mat color, cv::Mat depth, int *bin_con, int *bin_num, int nTargetItemIdx, std::vector<cv::Point>& grasp, std::vector<Hyp>& hyp, int viewpoint ){

	/* �e�w�̏�ݍ��݂ƃv�[�����O�̃p�����[�^ */
	CONV_POOLING_PARAMS conv_p[NUM_OF_CONV_LAYER];

	/* �p�����[�^�̏����� */
	for(int cc = 0; cc < NUM_OF_CONV_LAYER; cc++){
		conv_p[cc].conv_stride = 0;
		conv_p[cc].conv_padding = 0;
		conv_p[cc].pooling_stride = 0;
		conv_p[cc].pooling_size = 0;
	}

	/* 1�w�ڂ̏�ݍ��݂ƃv�[�����O�̃p�����[�^ */
	conv_p[0].conv_stride		= 4;
	conv_p[0].conv_padding		= 0;
	conv_p[0].pooling_stride	= 2;
	conv_p[0].pooling_size		= 3;
	conv_p[0].batch_narmarization = true;

	/* 2�w�ڂ̏�ݍ��݂ƃv�[�����O�̃p�����[�^ */
	conv_p[1].conv_stride		= 1;
	conv_p[1].conv_padding		= 2;
	conv_p[1].pooling_stride	= 2;
	conv_p[1].pooling_size		= 3;
	conv_p[1].batch_narmarization = true;

	/* 3�w�ڂ̏�ݍ��݂ƃv�[�����O�̃p�����[�^ */
	conv_p[2].conv_stride		= 1;
	conv_p[2].conv_padding		= 1;
	conv_p[2].pooling_stride	= 0;
	conv_p[2].pooling_size		= 0;
	conv_p[2].batch_narmarization = false;

	/* 4�w�ڂ̏�ݍ��݂ƃv�[�����O�̃p�����[�^ */
	conv_p[3].conv_stride		= 1;
	conv_p[3].conv_padding		= 1;
	conv_p[3].pooling_stride	= 0;
	conv_p[3].pooling_size		= 0;
	conv_p[3].batch_narmarization = false;

	/* 5�w�ڂ̏�ݍ��݂ƃv�[�����O�̃p�����[�^ */
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

			   printf("%d�w�ڂ̏�ݍ��݂ƃv�[�����O�̃p�����[�^��ݒ肵�Ă��������B\n", cc + 1);
			   getchar();
			   return false;

		}

	}

	cv::Mat item_idx = cv::Mat::zeros(cv::Size(40, 1), CV_32S);

	int total_items = *bin_num;
	for(int index = 0; index < total_items; index++){
		item_idx.at<int>(0, bin_con[index]) = 1;
	}

	/* �w�i��F���������Ȃ��ꍇ�C�ȉ����R�����g�A�E�g */
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

	/* ��ݍ��ݑw�̏d�݃t�B���^, �o�C�A�X, �����}�b�v�̃N���X */
	std::vector<CONVOLUTION_LAYER> conv_layer(NUM_OF_CONV_LAYER);

	/* �S�����w�̌����d��, �o�C�A�X, �����l���j�b�g�̃N���X */
	std::vector<FULL_CONECTION_LAYER> fc_layer(NUM_OF_FC_LAYER);

	/* �o�b�`�m�[�}���C�[�[�V�����̃��W���C���W��, ���ϒl, ���U�l�̃N���X */
	std::vector<BATCH_NORMALIZATION_LAYER> bn_layer(NUM_OF_BAT_NORM);

	printf("Loading convolution parameters...\n");
	for(int conv_num = 0; conv_num < NUM_OF_CONV_LAYER; conv_num++){

		/* ��ݍ��ݑw�̏d�݃t�B���^ */
		memset(conv_w_path, 0, sizeof(conv_w_path));
		sprintf(conv_w_path, "%s/conv%d_weight.npy", PARAM_DIR, conv_num + 1);

		/* ��ݍ��ݑw�̃o�C�A�X */
		memset(conv_b_path, 0, sizeof(conv_b_path));
		sprintf(conv_b_path, "%s/conv%d_bias.npy", PARAM_DIR, conv_num + 1);

		/* ��ݍ��ݑw�̏d�݃t�B���^�ƃo�C�A�X�̓ǂݍ��� */
		loadingConvolutionParams(conv_w_path, conv_b_path, conv_layer[conv_num].conv_w, conv_layer[conv_num].conv_b);

		/* �e�w�̏o�͓����}�b�v�� */
		int map_size = conv_layer[conv_num].conv_w.size();

		/* �e�w�̓����}�b�v�̃T�C�Y��ݒ� */
		conv_layer[conv_num].feature_map.resize(map_size);
		

	}

	printf("Loading fully conection parameters...\n");
	for(int fc_num = 0; fc_num < NUM_OF_FC_LAYER; fc_num++){

		/* �S�����w�̌����d�� */
		memset(fc_w_path, 0, sizeof(fc_w_path));
		sprintf(fc_w_path, "%s/l%d_weight.npy", PARAM_DIR, fc_num + 1);

		/* �S�����w�̃o�C�A�X */
		memset(fc_b_path, 0, sizeof(fc_b_path));
		sprintf(fc_b_path, "%s/l%d_bias.npy", PARAM_DIR, fc_num + 1);

		loadingFullConectionParams(fc_w_path, fc_b_path, fc_layer[fc_num].fc_w, fc_layer[fc_num].fc_b);

		/* �e�w�̏o�̓��j�b�g�� */
		int unit_size = fc_layer[fc_num].fc_w.size();

		/* �e�w�̃��j�b�g����ݒ� */
		fc_layer[fc_num].response_unit = cv::Mat::zeros(cv::Size(unit_size, 1), CV_32F);

	}

	printf("Loading batch normalization parameters...\n");
	for(int bn_num = 0; bn_num < NUM_OF_BAT_NORM; bn_num++){

		/* �o�b�`���K���̃��W�� */
		memset(bn_g_path, 0, sizeof(bn_g_path));
		sprintf(bn_g_path, "%s/bn%d_gamma.npy", PARAM_DIR, bn_num + 1);

		/* �o�b�`���K���̃��W�� */
		memset(bn_b_path, 0, sizeof(bn_b_path));
		sprintf(bn_b_path, "%s/bn%d_beta.npy", PARAM_DIR, bn_num + 1);

		/* �o�b�`���K���̕��ϒl */
		memset(bn_m_path, 0, sizeof(bn_m_path));
		sprintf(bn_m_path, "%s/bn%d_mean.dat", PARAM_DIR, bn_num + 1);

		/* �o�b�`���K���̕��U�l */
		memset(bn_v_path, 0, sizeof(bn_v_path));
		sprintf(bn_v_path, "%s/bn%d_var.dat", PARAM_DIR, bn_num + 1);

		loadingBatchNormalizationParams(bn_g_path, bn_b_path, bn_m_path, bn_v_path, bn_layer[bn_num].bn_g, bn_layer[bn_num].bn_b, bn_layer[bn_num].bn_mean, bn_layer[bn_num].bn_var);

	}

	printf("Loaded all parameters\n");

	/* ---��ݍ��ݑw�̊e�T�C�Y------------------------------------------- */
	/*                                                                    */
	/* i�w�ڂ̏o�͓����}�b�v��       -> conv_layer[i].conv_w.size()       */
	/* i�w�ڂ̓��͓����}�b�v��       -> conv_layer[i].conv_w[0].size()    */
	/* i�w�ڂ̏d�݃t�B���^�̏c�T�C�Y -> conv_layer[i].conv_w[0][0].rows   */
	/* i�w�ڂ̏d�݃t�B���^�̉��T�C�Y -> conv_layer[i].conv_w[0][0].cols   */
	/* i�w�ڂ̓����}�b�v��           -> conv_layer[i].feature_map.size()  */
	/* i�w�ڂ̓����}�b�v�̏c�T�C�Y   -> conv_layer[i].feature_map[0].rows */
	/* i�w�ڂ̓����}�b�v�̉��T�C�Y   -> conv_layer[i].feature_map[0].cols */
	/*                                                                    */
	/* ------------------------------------------------------------------ */

	/* ---�S�����w�̊e�T�C�Y------------------------------ */
	/*                                                     */
	/* i�w�ڂ̏o�̓��j�b�g�� -> fc_layer[i].fc_w.size()    */
	/* i�w�ڂ̓��̓��j�b�g�� -> fc_layer[i].fc_w[0].cols   */
	/*                                                     */
	/* --------------------------------------------------- */


	/* ���邳�␳�p�̃��b�N�A�b�v�e�[�u�� */
	unsigned char lut[256];

	/* ���邳�̕ϐ� 1�ȏ�F���邭�Ȃ�, 1�����F�Â��Ȃ� */
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

	/* ���b�N�A�b�v�e�[�u���̌v�Z */
	double gm = 1.0 / Gamma;
	for(int n = 0; n < 256; n++){
		lut[n] = pow(1.0 * (double)n / 255.0, gm) * 255.0;
	}


	/* �c���_���ӗ̈��؂�o�����J���[�p�b�`�摜 */
	cv::Mat color_patch;

	/* �c���_���ӗ̈��؂�o�����p�b�`�摜�̃��T�C�Y�摜 */
	cv::Mat resize_patch;

	/* ���邳�␳�����p�b�`�摜 */
	cv::Mat ill_patch;

	/* �c���_�̔F�����x�� */
	std::vector<int> recg_label(grasp.size());

	/* �c���n�̔F���X�R�A */
	std::vector<float> recg_score(grasp.size());

	/* RGB�摜�z�� */
	std::vector<cv::Mat> rgb(3);

	/* RGBD�摜�̏����� */
	for(int i = 0; i < 3; i++){
		rgb[i] = cv::Mat::zeros(cv::Size(CNN_PATCH_SIZE, CNN_PATCH_SIZE), CV_32F);
	}

	/* �c���_�̐��������[�v */
	for(int gp = 0; gp < grasp.size(); gp++){

		printf("------------Grasping point : %d\n", gp + 1);

		/* �J���[�摜�̔c���_���ӗ̈��؂�o�� */
		cv::getRectSubPix(color, cv::Size(PATCH_SIZE, PATCH_SIZE), grasp[gp], color_patch);

		/* �摜���A�b�v�T���v�����O */
		cv::resize(color_patch, resize_patch, cv::Size(CNN_PATCH_SIZE, CNN_PATCH_SIZE), 0, 0, cv::INTER_CUBIC);

		/* �摜�̖��邳�␳ */
		cv::LUT(resize_patch, cv::Mat(cv::Size(256, 1), CV_8U, lut), ill_patch);

		/* RGB�摜�̐��� */
		for(int y = 0; y < resize_patch.rows; y++){
			for(int x = 0; x < resize_patch.cols; x++){
				rgb[0].at<float>(y, x) = (float)ill_patch.data[resize_patch.step * y + resize_patch.channels() * x + 0] / 255.0;
				rgb[1].at<float>(y, x) = (float)ill_patch.data[resize_patch.step * y + resize_patch.channels() * x + 1] / 255.0;
				rgb[2].at<float>(y, x) = (float)ill_patch.data[resize_patch.step * y + resize_patch.channels() * x + 2] / 255.0;
			}
		}




		bool convolution_res, fullconection_res, activation_res, pooling_res, batch_res;


		/* ��ݍ��ݑw�̏��� */
		for(int conv_num = 0; conv_num < NUM_OF_CONV_LAYER; conv_num++){

			printf("��ݍ��ݏ���%d�w��\n", conv_num + 1);

			/* 1�w�ڂ͓��͉摜����ݍ��ݑw�ɓ���, 2�w�ڈȍ~��1�O�̑w�̓����}�b�v����� */
			if(conv_num == 0){	
				/* ��ݍ��ݏ��� */
				convolution_res = convolutionLayer(rgb, conv_layer[conv_num].feature_map, conv_layer[conv_num].conv_w, conv_layer[conv_num].conv_b, conv_p[conv_num].conv_stride, conv_p[conv_num].conv_padding);
				/* �o�b�`���K�� */
				if(conv_p[conv_num].batch_narmarization){
					batch_res = batchNormalization(conv_layer[conv_num].feature_map, bn_layer[conv_num].bn_g, bn_layer[conv_num].bn_b, bn_layer[conv_num].bn_mean, bn_layer[conv_num].bn_var);
				}
				/* �������֐� */
				activation_res = convReLU(conv_layer[conv_num].feature_map);
				/* �v�[�����O */
				if(conv_p[conv_num].pooling_size != 0){
					pooling_res = maxPooling(conv_layer[conv_num].feature_map, conv_p[conv_num].pooling_stride, conv_p[conv_num].pooling_size);
				}

				if(!convolution_res){
					printf("��ݍ��ݎ��s�B\n");
					return false;
				}
			}
			else{
				/* ��ݍ��ݏ��� */
				convolution_res = convolutionLayer(conv_layer[conv_num - 1].feature_map, conv_layer[conv_num].feature_map, conv_layer[conv_num].conv_w, conv_layer[conv_num].conv_b, conv_p[conv_num].conv_stride, conv_p[conv_num].conv_padding);
				/* �o�b�`���K�� */
				if(conv_p[conv_num].batch_narmarization){
					batch_res = batchNormalization(conv_layer[conv_num].feature_map, bn_layer[conv_num].bn_g, bn_layer[conv_num].bn_b, bn_layer[conv_num].bn_mean, bn_layer[conv_num].bn_var);
				}
				/* �������֐� */
				activation_res = convReLU(conv_layer[conv_num].feature_map);
				/* �v�[�����O */
				if(conv_p[conv_num].pooling_size != 0){
					pooling_res = maxPooling(conv_layer[conv_num].feature_map, conv_p[conv_num].pooling_stride, conv_p[conv_num].pooling_size);
				}

				if(!convolution_res){
					printf("��ݍ��ݎ��s�B\n");
					return false;
				}
			}

		}

		int conv_output_size = conv_layer[NUM_OF_CONV_LAYER - 1].feature_map.size() * conv_layer[NUM_OF_CONV_LAYER - 1].feature_map[0].rows * conv_layer[NUM_OF_CONV_LAYER - 1].feature_map[0].cols;

		/* ��ݍ��ݍŏI�w�̔z�� */
		cv::Mat conv_output = cv::Mat::zeros(cv::Size(conv_output_size, 1), CV_32F);

		/* ��ݍ��ݍŏI�w�̑S�����}�b�v��1�����ɕϊ� */
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
			printf("��ݍ��݂̏o�͎�����(%d)�ƑS�����̓��͎�����(%d)����v���܂���D\n", conv_output.cols, fc_layer[0].fc_w[0].cols);
			return false;
		}


		/* �N���X�m�� */
		cv::Mat probability;

		/* �S�����w�̏��� */
		for(int fc_num = 0; fc_num < NUM_OF_FC_LAYER; fc_num++){

			printf("�S��������%d�w��\n", fc_num + 1);

			/* 1�w�ڂ͏�ݍ��ݑw�̏o�͂�S�����w�ɓ���, 2�w�ڈӌ���1�O�̑w�̉����l���j�b�g�����, �ŏI�w�̊������֐���Softmax */
			if(fc_num == 0){
				/* �S�������� */
				fullconection_res = fullConectionLayer(conv_output, fc_layer[fc_num].response_unit, fc_layer[fc_num].fc_w, fc_layer[fc_num].fc_b);
				/* �������֐� */
				activation_res = fcReLU(fc_layer[fc_num].response_unit);
			}
			else if(fc_num == NUM_OF_FC_LAYER - 1){
				/* �S�������� */
				fullconection_res = fullConectionLayer(fc_layer[fc_num - 1].response_unit, fc_layer[fc_num].response_unit, fc_layer[fc_num].fc_w, fc_layer[fc_num].fc_b);
				/* �ʏ��SoftMax�֐� */
				//activation_res = softMax(fc_layer[fc_num].response_unit, probability);
				/* ����t��SoftMax�֐� */
				activation_res = conditionalSoftMax(fc_layer[fc_num].response_unit, probability, item_idx);
			}
			else{
				/* �S�������� */
				fullconection_res = fullConectionLayer(fc_layer[fc_num - 1].response_unit, fc_layer[fc_num].response_unit, fc_layer[fc_num].fc_w, fc_layer[fc_num].fc_b);
				/* �������֐� */
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


	/* -----�����ȉ�vector�z��̃N���A------------------------------------------------ */

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
