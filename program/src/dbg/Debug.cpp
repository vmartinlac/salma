#include <QVBoxLayout>
#include <QScrollArea>
#include <QDialog>
#include <QPixmap>
#include <QLabel>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include "Debug.h"

void Debug::imshow(const cv::Mat& original_image)
{
    cv::Mat image;
    cv::cvtColor(original_image, image, CV_BGR2RGB);

    QImage im;

    switch(image.type())
    {
    case CV_8UC3:
        im = QImage(image.ptr(0), image.cols, image.rows, QImage::Format_RGB888);
        break;
    case CV_8UC1:
        im = QImage(image.ptr(0), image.cols, image.rows, QImage::Format_Grayscale8);
        break;
    default:
        throw("incorrect image format");
    }

    const int max_width = 1500;
    if( im.width() > max_width )
    {
        im = im.scaledToWidth(max_width);
    }

    QLabel* l = new QLabel();
    l->setPixmap(QPixmap::fromImage(im));

    QScrollArea* area = new QScrollArea();
    area->setWidget(l);
    area->setAlignment(Qt::AlignCenter);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->setAlignment(Qt::AlignCenter);
    lay->addWidget(area);

    QDialog* dlg = new QDialog();
    dlg->setLayout(lay);
    dlg->exec();

    delete dlg;
}

void Debug::plotkeypointsandmatches(
    const cv::Mat& left,
    const std::vector<cv::KeyPoint>& lkpts,
    const cv::Mat& right,
    const std::vector<cv::KeyPoint>& rkpts,
    const std::vector< std::pair<int,int> >& matches)
{
    std::vector<cv::DMatch> m(matches.size());

    std::transform( matches.cbegin(), matches.cend(), m.begin(), [] ( const std::pair<int,int>& pair )
    {
        return cv::DMatch( pair.first, pair.second, 1.0 );
    });

    cv::Mat out;
    cv::drawMatches( left, lkpts, right, rkpts, m, out);

    imshow(out);    
}

