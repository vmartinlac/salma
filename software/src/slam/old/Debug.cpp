#include <QVBoxLayout>
#include <QScrollArea>
#include <QDialog>
#include <QPixmap>
#include <QLabel>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include "Debug.h"

void Debug::imshow(const cv::Mat& image)
{
    cv::Mat true_image;

    if(image.type() == CV_8UC3 )
    {
        cv::cvtColor(image, true_image, cv::COLOR_BGR2RGB);
    }
    else if(image.type() == CV_8UC1 )
    {
        true_image = image;
    }
    else if(image.type() == CV_16SC1)
    {
        cv::Mat A;
        cv::normalize(image, A, 255.0, 0.0, cv::NORM_MINMAX);
        A.convertTo(true_image, CV_8UC1);
    }
    else if(image.type() == CV_32FC1)
    {
        cv::Mat A;
        cv::normalize(image, A, 255.0, 0.0, cv::NORM_MINMAX);
        A.convertTo(true_image, CV_8UC1);
    }
    else
    {
        throw std::runtime_error("internal error");
    }

    QImage im;

    switch(true_image.type())
    {

    case CV_8UC3:
        im = QImage(true_image.ptr(0), true_image.cols, true_image.rows, true_image.step, QImage::Format_RGB888);
        break;

    case CV_8UC1:
        im = QImage(true_image.ptr(0), true_image.cols, true_image.rows, true_image.step, QImage::Format_Grayscale8);
        break;

    default:
        throw std::runtime_error("internal error");
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

void Debug::stereoimshow(
    const cv::Mat& left,
    const cv::Mat& right,
    const std::vector<cv::KeyPoint>& lkpts,
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

