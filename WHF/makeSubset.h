#pragma once

#include "WHFcommon.h"
#include "sample.h"
using namespace cv;

class makeSubset
{
public:
	vector<testSample> testPaches;
	vector<testSample> extractPatch( Mat &testImage );
	makeSubset(void);
	~makeSubset(void);
};

