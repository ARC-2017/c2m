
/*‘SŒ‹‡ˆ—ŠÖ” */
bool fullConectionLayer(cv::Mat input_unit, cv::Mat &output_unit, std::vector<cv::Mat> fc_w, std::vector<float> bias);

/* ReLUŠÖ” */
bool fcReLU(cv::Mat &response_unit);

/* SoftmaxŠÖ” */
bool softMax(cv::Mat response_unit, cv::Mat &output_unit);

/* §–ñ•t‚«SoftmaxŠÖ” */
bool conditionalSoftMax(cv::Mat response_unit, cv::Mat &output_unit, cv::Mat item_idx);