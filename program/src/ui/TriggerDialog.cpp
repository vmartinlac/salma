#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include "TriggerDialog.h"

TriggerDialog::TriggerDialog(QWidget* parent)
{
    mArduinoButton = new QRadioButton("Trigger via Arduino");
    mSoftwareButton = new QRadioButton("Trigger via GenICam");
    mPath = new PathWidget(PathWidget::GET_OPEN_FILENAME, this);
    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");
    QVBoxLayout* vl = new QVBoxLayout();
    QHBoxLayout* hl = new QHBoxLayout();

    vl->addWidget(mArduinoButton);
    vl->addWidget(mSoftwareButton);
    vl->addWidget(mPath);

    hl->addWidget(btnok);
    hl->addWidget(btncancel);
    vl->addLayout(hl);

    setLayout(vl);
    setWindowTitle("Trigger Source");
}

void TriggerDialog::getTriggerInfo(TriggerInfo& trigger)
{
    if(mArduinoButton->isChecked())
    {
      trigger.mode = TRIGGER_ARDUINO;
      trigger.path = mPath->path().toStdString();
    }
    //else if(mSoftwareButton->isChecked())
    else
    {
      trigger.mode = TRIGGER_SOFTWARE;
      trigger.path.clear();
    }
}

