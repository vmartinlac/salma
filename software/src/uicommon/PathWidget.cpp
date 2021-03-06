#include <QHBoxLayout>
#include <QFileDialog>
#include <QPushButton>
#include "PathWidget.h"

PathWidget::PathWidget(Mode mode, QWidget* parent) : QWidget(parent)
{
    mMode = mode;
    mText = new QLineEdit();

    QPushButton* btn = new QPushButton("Select");

    QHBoxLayout* lay = new QHBoxLayout();
    lay->setContentsMargins(0, 0, 0, 0);
    lay->addWidget(mText, 1);
    lay->addWidget(btn, 0);

    setLayout(lay);

    QObject::connect(btn, SIGNAL(clicked()), this, SLOT(selectFile()));
}

void PathWidget::selectFile()
{
    QString ret;

    switch(mMode)
    {
    case GET_EXISTING_DIRECTORY:
        ret = QFileDialog::getExistingDirectory(this, "Select directory", mText->text());
        break;
    case GET_SAVE_FILENAME:
        ret = QFileDialog::getSaveFileName(this, "Select output file", mText->text());
        break;
    case GET_OPEN_FILENAME:
        ret = QFileDialog::getOpenFileName(this, "Select input file", mText->text());
        break;
    default:
        throw std::runtime_error("Internal error");
    }

    if(ret.isEmpty() == false)
    {
        setPath(ret);
    }
}

void PathWidget::setPath(const QString& str)
{
    mText->setText(str);
}

QString PathWidget::path()
{
    return mText->text();
}

