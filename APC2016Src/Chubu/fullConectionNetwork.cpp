#include "header_cnn.h"
#include "fullConectionNetwork.h"


/*全結合処理関数 */
bool fullConectionLayer(cv::Mat input_unit, cv::Mat &output_unit, std::vector<cv::Mat> fc_w, std::vector<float> bias){

	int output_channel = fc_w.size();
	int input_channel = fc_w[0].cols;

	/* 出力ユニットのループ */
	for(int och = 0; och < output_channel; och++){

		cv::Mat conbination_weight = fc_w[och];

		float fc_val = 0.0;

		/* 入力ユニットのループ */
		for(int ich = 0; ich < input_channel; ich++){

			fc_val += input_unit.at<float>(0, ich) * conbination_weight.at<float>(0, ich);

		}

		output_unit.at<float>(0, och) = fc_val + bias[och];

	}

	return true;
}

/* ReLU関数 */
bool fcReLU(cv::Mat &response_unit){

	response_unit = cv::max(0,response_unit);

	return true;
}

/* Softmax関数 */
bool softMax(cv::Mat response_unit, cv::Mat &output_unit){

	cv::Mat exp_mat;
	cv::Mat sum_mat;

	cv::exp(response_unit, exp_mat);
	cv::reduce(exp_mat, sum_mat, 1, CV_REDUCE_SUM);

	output_unit = exp_mat / sum_mat.at<float>(0, 0);

	return true;

}

/* 制約付きSoftmax関数 */
bool conditionalSoftMax(cv::Mat response_unit, cv::Mat &output_unit, cv::Mat item_idx){

	for(int i = 0; i < item_idx.cols; i++){
		if(item_idx.at<int>(0, i) == 0){
			response_unit.at<float>(0, i) = -9999.0;
		}
	}

	cv::Mat exp_mat;
	cv::Mat sum_mat;

	cv::exp(response_unit, exp_mat);
	cv::reduce(exp_mat, sum_mat, 1, CV_REDUCE_SUM);

	output_unit = exp_mat / sum_mat.at<float>(0, 0);

	return true;

}