
/*�S���������֐� */
bool fullConectionLayer(cv::Mat input_unit, cv::Mat &output_unit, std::vector<cv::Mat> fc_w, std::vector<float> bias);

/* ReLU�֐� */
bool fcReLU(cv::Mat &response_unit);

/* Softmax�֐� */
bool softMax(cv::Mat response_unit, cv::Mat &output_unit);

/* ����t��Softmax�֐� */
bool conditionalSoftMax(cv::Mat response_unit, cv::Mat &output_unit, cv::Mat item_idx);