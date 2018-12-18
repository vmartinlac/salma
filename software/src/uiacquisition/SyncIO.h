#pragma once

#include <opencv2/core.hpp>
#include <vector>

bool syncimwrite(const std::string& path, const std::string& ext, const cv::Mat& image);

