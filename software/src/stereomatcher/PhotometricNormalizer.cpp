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

    mRightToLeft.create(3, 256, CV_8UC1);

    for(int i=0; i<3; i++)
    {
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

        for(int j=0; j<256; j++)
        {
            // TODO
            // mRightToLeft.at<uint8_t>(i, j) = 
        }
    }
}

void PhotometricNormalizer::operator()(
    const cv::Mat& left_from,
    const cv::Mat& right_from,
    cv::Mat& left_to,
    cv::Mat& right_to)
{
    if( left_from.type() != CV_8UC3 ) throw std::runtime_error("incorrect mat type!");

    if( right_from.type() != CV_8UC3 ) throw std::runtime_error("incorrect mat type!");

    auto proc_right = [this] (const cv::Vec3b& from)
    {
        uint8_t red = mRightToLeft.at<uint8_t>(0, from[0]);
        uint8_t green = mRightToLeft.at<uint8_t>(1, from[1]);
        uint8_t blue = mRightToLeft.at<uint8_t>(2, from[2]);
        return cv::Vec3b(red, green, blue);
    };

    left_to = left_from.clone();

    right_to.create( right_from.rows, right_from.cols, CV_8UC3 );
    std::transform( right_from.begin<cv::Vec3b>(), right_from.end<cv::Vec3b>(), right_to.begin<cv::Vec3b>(), proc_right );
}

