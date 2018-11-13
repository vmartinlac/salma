#include <QVBoxLayout>
#include <QDialog>
#include <QPixmap>
#include <QLabel>
#include <opencv2/features2d.hpp>
#include "Debug.h"

void Debug::imshow(const cv::Mat& image)
{
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

    if( im.width() > 900 )
    {
        im = im.scaledToWidth(900);
    }

    QLabel* l = new QLabel();
    l->setPixmap(QPixmap::fromImage(im));

    QVBoxLayout* lay = new QVBoxLayout();
    lay->setAlignment(Qt::AlignCenter);
    lay->addWidget(l);

    QDialog* dlg = new QDialog();
    dlg->setLayout(lay);
    dlg->exec();

    delete dlg;
}

void Debug::plotkeypointsandmatches(
    const cv::Mat& left,
    const std::vector<cv::KeyPoint>& lkpts,
    const cv::Mat& right,
    const std::vector<cv::KeyPoint>& rkpts)
{
    std::vector<cv::DMatch> nada;
    cv::Mat out;
    cv::drawMatches(
        left, lkpts, right, rkpts, nada, out);

    imshow(out);    
}

