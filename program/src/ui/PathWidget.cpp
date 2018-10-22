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
    lay->addWidget(mText);
    lay->addWidget(btn);

    setLayout(lay);
}

void PathWidget::selectFile()
{
    QString ret;

    switch(mMode)
    {
    case GET_EXISTING_DIRECTORY:
        ret = QFileDialog::getExistingDirectory(this, "Select directory");
        break;
    case GET_SAVE_FILENAME:
        ret = QFileDialog::getSaveFileName(this, "Select directory");
        break;
    case GET_OPEN_FILENAME:
    default:
        ret = QFileDialog::getOpenFileName(this, "Select file");
        break;
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

