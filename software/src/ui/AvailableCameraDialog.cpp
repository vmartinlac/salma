#include <QTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <sstream>
#include "AvailableCameraDialog.h"
#include "VideoSystem.h"

AvailableCameraDialog::AvailableCameraDialog(QWidget* p) : QDialog(p)
{
    VideoSystem* vid = VideoSystem::instance();
    if(vid == nullptr) throw std::runtime_error("Unitialized video system!");

    std::stringstream s;
    s << "<html><head></head><body>";
    s << "<h1>Available Cameras</h1>";
    if( vid->getNumberOfGenICamCameras() > 0 )
    {
        s << "<ol>";
        for(int i=0; i<vid->getNumberOfGenICamCameras(); i++)
        {
            s << "<li>" << vid->getNameOfGenICamCamera(i) << "</li>";
        }
        s << "</ol>";
    }
    else
    {
        s << "<p>No camera detected!</p>";
    }
    s << "</body></html>";

    QTextEdit* t = new QTextEdit();
    t->setText(s.str().c_str());
    t->setReadOnly(true);

    QPushButton* btn = new QPushButton("Close");

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(t);
    lay->addWidget(btn);

    setWindowTitle("Available cameras");
    setLayout(lay);

    connect(btn, SIGNAL(clicked()), this, SLOT(accept()));
}

