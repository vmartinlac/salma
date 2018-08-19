#include <QVBoxLayout>
#include <QButtonGroup>
#include <QFileInfo>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QPushButton>
#include <QFormLayout>
#include "VideoInputDialog.h"

VideoInputDialog::VideoInputDialog(QWidget* parent) : QDialog(parent)
{
    create_avt_frame();
    create_opencv_frame();
    create_file_frame();

    QVBoxLayout* lay = new QVBoxLayout();

    m_avt = new QRadioButton("Allied Vision Technologies camera");
    m_opencv = new QRadioButton("OpenCV camera");
    m_file = new QRadioButton("Video file");

    QButtonGroup* grp = new QButtonGroup(this);
    grp->addButton(m_avt);
    grp->addButton(m_opencv);
    grp->addButton(m_file);

    lay->addWidget(m_avt);
    lay->addWidget(m_avt_frame);
    lay->addWidget(m_opencv);
    lay->addWidget(m_opencv_frame);
    lay->addWidget(m_file);
    lay->addWidget(m_file_frame);

    QPushButton* btn_ok = new QPushButton("OK");
    QPushButton* btn_cancel = new QPushButton("Cancel");

    QHBoxLayout* hlay = new QHBoxLayout();
    hlay->addWidget(btn_ok);
    hlay->addWidget(btn_cancel);
    lay->addLayout(hlay);

    setLayout(lay);
    setWindowTitle("Video Input");

    connect(btn_ok, SIGNAL(clicked()), this, SLOT(accept()));
    connect(btn_cancel, SIGNAL(clicked()), this, SLOT(reject()));
    connect(grp, SIGNAL(buttonClicked(int)), this, SLOT(input_changed()));
}

void VideoInputDialog::create_avt_frame()
{
    m_avt_camera = new QComboBox();

    QFormLayout* form = new QFormLayout();
    form->addRow("Camera:", m_avt_camera);

    m_avt_frame = new QFrame();
    m_avt_frame->setLayout(form);
}

void VideoInputDialog::create_opencv_frame()
{
    m_opencv_camera = new QSpinBox();

    QFormLayout* form = new QFormLayout();
    form->addRow("Camera id:", m_opencv_camera);

    m_opencv_frame = new QFrame();
    m_opencv_frame->setLayout(form);
}

void VideoInputDialog::create_file_frame()
{
    m_file_choose = new QPushButton("Choose...");

    QFormLayout* form = new QFormLayout();
    form->addRow("File:", m_file_choose);

    m_file_frame = new QFrame();
    m_file_frame->setLayout(form);

    connect(m_file_choose, SIGNAL(clicked()), this, SLOT(choose_file()));
}

void VideoInputDialog::choose_file()
{
    QString ret = QFileDialog::getOpenFileName(this, "Open Video File");

    if(ret.isEmpty() == false)
    {
        QFileInfo file(ret);
        m_file_choose->setText(file.fileName());
    }
}

void VideoInputDialog::input_changed()
{
    m_avt_frame->setEnabled( m_avt->isChecked() );
    m_file_frame->setEnabled( m_file->isChecked() );
    m_opencv_frame->setEnabled( m_opencv->isChecked() );
}

void VideoInputDialog::accept()
{
    if(false) QDialog::accept();
}

