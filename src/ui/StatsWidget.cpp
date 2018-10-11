#include <QPalette>
#include <QFont>
#include "StatsWidget.h"

StatsWidget::StatsWidget(QWidget* parent) : QPlainTextEdit(parent)
{
    mPort = new StatsInputPort(this);

    QPalette p = palette();
    p.setColor(QPalette::Base, Qt::black);
    p.setColor(QPalette::Text, Qt::white);
    setPalette(p);

    QFont myfont = font();
    myfont.setStyleHint(QFont::Monospace);
    myfont.setBold(true);
    setFont(myfont);

    setReadOnly(true);

    QObject::connect(mPort, SIGNAL(updated()), this, SLOT(refresh()));
}

StatsInputPort* StatsWidget::getPort()
{
    return mPort;
}

void StatsWidget::refresh()
{
    StatsInputData data;
    mPort->read(data);

    setPlainText(data.text);
}

