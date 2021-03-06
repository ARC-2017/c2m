
/*全結合処理関数 */
bool fullConectionLayer(cv::Mat input_unit, cv::Mat &output_unit, std::vector<cv::Mat> fc_w, std::vector<float> bias);

/* ReLU関数 */
bool fcReLU(cv::Mat &response_unit);

/* Softmax関数 */
bool softMax(cv::Mat response_unit, cv::Mat &output_unit);

/* 制約付きSoftmax関数 */
bool conditionalSoftMax(cv::Mat response_unit, cv::Mat &output_unit, cv::Mat item_idx);