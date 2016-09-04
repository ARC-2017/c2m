#include "header_cnn.h"
#include "numpy.hpp"
#include "loading_params.h"

bool loadingConvolutionParams(char conv_w_path[256], char conv_b_path[256], std::vector< std::vector<cv::Mat> > &conv_w, std::vector<float> &bias){

	std::vector<int> data_size;
	std::vector<float> data;

	std::vector<int> bias_size;

	aoba::LoadArrayFromNumpy(conv_w_path, data_size, data);
	aoba::LoadArrayFromNumpy(conv_b_path, bias_size, bias);

	conv_w = std::vector< std::vector<cv::Mat> >(data_size[0], std::vector<cv::Mat>(data_size[1]));

	size_t cnt = 0;
	for(int out_ch = 0; out_ch < data_size[0]; out_ch++){
		for(int in_ch = 0; in_ch < data_size[1]; in_ch++){
			conv_w[out_ch][in_ch] = cv::Mat::zeros(cv::Size(data_size[2], data_size[3]), CV_32F);

			for(int j = 0; j < data_size[3]; j++){
				for(int i = 0; i < data_size[2]; i++){
					conv_w[out_ch][in_ch].at<float>(j, i) = data[cnt];
					cnt++;
				}
			}

		}
	}

	data_size.clear();
	bias_size.clear();
	data.clear();

	std::vector<int> ().swap(data_size);
	std::vector<int> ().swap(bias_size);
	std::vector<float> ().swap(data);

	return true;
}

bool loadingFullConectionParams(char fc_w_path[256], char fc_b_path[256], std::vector<cv::Mat> &fc_w, std::vector<float> &bias){

	std::vector<int> data_size;
	std::vector<float> data;

	std::vector<int> bias_size;

	aoba::LoadArrayFromNumpy(fc_w_path, data_size, data);
	aoba::LoadArrayFromNumpy(fc_b_path, bias_size, bias);

	fc_w = std::vector<cv::Mat>(data_size[0]);

	size_t cnt = 0;
	for(int out_ch = 0; out_ch < data_size[0]; out_ch++){

		fc_w[out_ch] = cv::Mat::zeros(cv::Size(data_size[1], 1), CV_32F);

		for(int in_ch = 0; in_ch < data_size[1]; in_ch++){

			fc_w[out_ch].at<float>(0, in_ch) = data[cnt];
			cnt++;

		}
	}

	data_size.clear();
	bias_size.clear();
	data.clear();

	std::vector<int> ().swap(data_size);
	std::vector<int> ().swap(bias_size);
	std::vector<float> ().swap(data);

	return true;

}

bool loadingBatchNormalizationParams(char bn_g_path[256], char bn_b_path[256], char bn_m_path[256], char bn_v_path[256], std::vector<float> &gamma, std::vector<float> &beta, std::vector<float> &mean, std::vector<float> &var){

	std::vector<int> gamma_size;
	std::vector<int> beta_size;
	
	aoba::LoadArrayFromNumpy(bn_g_path, gamma_size, gamma);
	aoba::LoadArrayFromNumpy(bn_b_path, beta_size, beta);

	int cnt;

	std::string m_char, v_char;
	float m, v;

	std::ifstream mfs(bn_m_path);
	std::ifstream vfs(bn_v_path);

	if(!mfs){
		printf("Batch Normalizationの平均値が読み込めません。(%s)\n", bn_m_path);
		return false;
	}

	if(!vfs){
		printf("Batch Normalizationの分散値が読み込めません。(%s)", bn_v_path);
		return false;
	}

	mean.resize(gamma.size());
	var.resize(gamma.size());

	cnt = 0;
	while(std::getline(mfs, m_char)){
		m = std::stof(m_char);
		mean[cnt] = m;
		cnt++;
	}

	cnt = 0;
	while(std::getline(vfs, v_char)){
		v = std::stof(v_char);
		var[cnt] = v;
		cnt++;
	}

	gamma_size.clear();
	beta_size.clear();

	std::vector<int> ().swap(gamma_size);
	std::vector<int> ().swap(beta_size);

	return true;
}