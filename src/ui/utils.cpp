#include <iostream>
#include <QScrollArea>
#include <QLabel>
#include <QImage>
#include <QVBoxLayout>
#include <QDialog>
#include <QColor>
#include "utils.h"

namespace ui {

void imshow(const cv::Mat& mat)
{
    QImage image(mat.cols, mat.rows, QImage::Format_RGB888);
    for(int i=0; i<image.width(); i++)
    {
        for(int j=0; j<image.height(); j++)
        {
            QColor col;

            switch(mat.type())
            {
            case CV_8UC3:
                {
                    cv::Vec3d col2 = mat.at<cv::Vec3b>(j, i);
                    col = QColor( col2(0), col2(1), col2(2) );
                }
                break;

            case CV_32S:
                {
                    int32_t level = mat.at<int32_t>(j, i);
                    col = QColor( level, level, level );
                }
                break;

            case CV_32F:
                {
                    float level = mat.at<float>(j, i);
                    col = QColor( level, level, level );
                }
                break;

            case CV_8U:
                {
                    uint8_t level = mat.at<uint8_t>(j, i);
                    col = QColor( level, level, level );
                }
                break;

            default:
                std::cerr << mat.type() << std::endl;
                throw std::runtime_error("Bad type");
            }

            image.setPixel(i, j, col.rgb());
        }
    }

    QLabel* l = new QLabel();
    l->setPixmap(QPixmap::fromImage(image));

    QScrollArea* s = new QScrollArea();
    s->setWidget(l);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(s);

    QDialog* dlg = new QDialog();
    dlg->setLayout(lay);
    dlg->setWindowTitle("Image");

    dlg->exec();

    delete dlg;
}

}
