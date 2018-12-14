#include <iostream>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <opencv2/imgcodecs.hpp>
#include <Tracker.h>

int main(int num_args, char** args)
{
    QCoreApplication app(num_args, args);

    QCommandLineParser parser;
    parser.addPositionalArgument("IMAGE", "Path to the image to be processed");
    parser.addOption(QCommandLineOption("unit-length", "Length of the side of a square on the target", "UNIT_LENGTH", "1.0"));
    parser.addOption(QCommandLineOption("absolute-orientation", "Whether to compute absolute orientation or any orientation", "ABSOLUTE", "1"));
    parser.process(app);

    if( parser.positionalArguments().size() != 1 )
    {
        std::cout << "Incorrect number of arguments!" << std::endl;
        exit(1);
    }

    cv::Mat image = cv::imread(parser.positionalArguments().front().toLocal8Bit().data());
    if(image.data == nullptr)
    {
        std::cout << "Could not read input image!" << std::endl;
        exit(1);
    }

    target::Tracker t;
    t.setUnitLength(parser.value("unit-length").toDouble());
    t.track(image, (parser.value("absolute-orientation").toInt() != 0) );

    if(t.found())
    {
        std::cout << "Target found!" << std::endl;

        const int N = t.objectPoints().size();

        for(int i=0; i<N; i++)
        {
            std::cout << t.pointIds()[i] << ' ';
            std::cout << t.imagePoints()[i].x << ' ';
            std::cout << t.imagePoints()[i].y << ' ';
            std::cout << t.objectPoints()[i].x << ' ';
            std::cout << t.objectPoints()[i].y << ' ';
            std::cout << t.objectPoints()[i].z << std::endl;
        }
    }
    else
    {
        std::cout << "Target not found!" << std::endl;
    }

    return 0;
}
