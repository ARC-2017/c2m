#include"..\\Chukyo\\LabelOptimization_data.h"


bool RunCNN( cv::Mat color, cv::Mat depth, int *bin_con, int *bin_num, int nTargetItemIdx, std::vector<cv::Point>& grasp, std::vector<Hyp>& hyp, int viewpoint );