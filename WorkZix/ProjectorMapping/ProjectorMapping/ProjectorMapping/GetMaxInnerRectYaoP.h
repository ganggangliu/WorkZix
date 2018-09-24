#ifndef GET_MAX_INNER_RECT_YAOP
#define GET_MAX_INNER_RECT_YAOP

#include "OpenCVInc.h"
#include <vector>

cv::Rect getMaxInscribedRectangle(std::vector<cv::Point>& contours, double recommendRatio);

#endif