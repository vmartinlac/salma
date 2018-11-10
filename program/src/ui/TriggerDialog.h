#pragma once

#include <QDialog>
#include <QRadioButton>
#include "PathWidget.h"
#include "TriggerInfo.h"

class TriggerDialog : public QDialog
{
    Q_OBJECT

public:

    TriggerDialog(QWidget* parent=nullptr);
    void getTriggerInfo(TriggerInfo& trigger);

protected:

      PathWidget* mPath;
      QRadioButton* mSoftwareButton;
      QRadioButton* mArduinoButton;
};

