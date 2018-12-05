#include <QHBoxLayout>
#include <QPushButton>
#include "TriggerDialog.h"
#include "TriggerWidget.h"

TriggerWidget::TriggerWidget(QWidget* parent)
{
    mLabel = new QLabel();

    QPushButton* btn = new QPushButton("change");

    QHBoxLayout* lay = new QHBoxLayout();
    setLayout(lay);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->addWidget(mLabel, 1);
    lay->addWidget(btn, 0);

    updateLabel();
}

void TriggerWidget::updateLabel()
{
  if(mTrigger.mode == TRIGGER_SOFTWARE)
  {
    mLabel->setText("software");
  }
  else if(mTrigger.mode == TRIGGER_ARDUINO)
  {
    mLabel->setText("arduino on " + QString(mTrigger.path.c_str()));
  }
  else
  {
    mLabel->setText("N/A");
  }
}

void TriggerWidget::onButtonClicked()
{
    TriggerDialog* dlg = new TriggerDialog(this);
    const int ret = dlg->exec();
    if(ret == QDialog::Accepted)
    {
        dlg->getTriggerInfo(mTrigger);
        updateLabel();
    }
    delete dlg;
}

