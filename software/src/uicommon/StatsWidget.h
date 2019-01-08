#pragma once

#include <QPlainTextEdit>
#include "Port.h"

struct StatsInputData
{
    QString text;
};

typedef Port<StatsInputData> StatsInputPort;

class StatsWidget : public QPlainTextEdit
{
    Q_OBJECT

public:

    StatsWidget(QWidget* parent=nullptr);

    StatsInputPort* getPort();

protected slots:

    void refresh();

protected:

    StatsInputPort* mPort;
};

