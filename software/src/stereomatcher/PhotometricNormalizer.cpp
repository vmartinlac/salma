#include "PhotometricNormalizer.h"


PhotometricNormalizer::PhotometricNormalizer()
{
}

void PhotometricNormalizer::prepare(const cv::Mat& left_image, const cv::Mat& right_image)
{
    if(left_image.size() != right_image.size()) throw std::runtime_error("left and right images differ in size!");

    if(left_image.type() != CV_8UC3 || right_image.type() != CV_8UC3) throw std::runtime_error("invalid image format");

    const int margin = std::min(left_image.cols, left_image.rows)/6;

    cv::Rect roi(margin, margin, left_image.cols-margin, left_image.rows-margin);

    const cv::Mat left_roi = left_image(roi);
    const cv::Mat right_roi = right_image(roi);

    for(int i=0; i<3; i++)
    {
        mRightToLeft[i].clear();

        std::vector<uint8_t> left_data;
        std::vector<uint8_t> right_data;

        std::transform(
            left_roi.begin<cv::Vec3b>(),
            left_roi.end<cv::Vec3b>(),
            std::back_inserter(left_data),
            [i] (const cv::Vec3b& rgb) { return rgb(i); } );

        std::transform(
            right_roi.begin<cv::Vec3b>(),
            right_roi.end<cv::Vec3b>(),
            std::back_inserter(right_data),
            [i] (const cv::Vec3b& rgb) { return rgb(i); } );

        std::sort( left_data.begin(), left_data.end() );

        std::sort( right_data.begin(), right_data.end() );

        mRightToLeft[i].resize(256);

        for(int i=0; i<256; i++)
        {
            // TODO
        }
    }
}

