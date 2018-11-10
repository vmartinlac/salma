#pragma once

#include <QWidget>
#include <QLabel>
#include "TriggerInfo.h"

class TriggerWidget : public QWidget
{
    Q_OBJECT

public:

    TriggerWidget(QWidget* parent = nullptr);

protected slots:

    void updateLabel();
    void onButtonClicked();

protected:

    TriggerInfo mTrigger;
    QLabel* mLabel;
};

