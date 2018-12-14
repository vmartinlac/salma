#include <QFrame>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include "OperationDialog.h"

#include "CameraCalibrationParametersWidget.h"
#include "StereoRigCalibrationParametersWidget.h"
#include "MonoRecordingParametersWidget.h"
#include "StereoRecordingParametersWidget.h"

OperationDialog::OperationDialog(QWidget* parent) : QDialog(parent)
{
    createOperationParametersWidgets();

    if( mOperationParametersWidgets.empty() )
    {
        throw std::runtime_error("mOperationParametersWidgets should not be empty!");
    }

    mPageHome = createHomeWidget();

    mStackedWidget = new QStackedWidget();
    mStackedWidget->addWidget(mPageHome);

    for(OperationParametersWidget* w : mOperationParametersWidgets)
    {
        mStackedWidget->addWidget(w);
    }

    mStackedWidget->setCurrentWidget(mPageHome);

    QFrame* frame = new QFrame();
    frame->setFrameShape(QFrame::HLine);

    QPushButton* btnok = new QPushButton("OK");
    QPushButton* btncancel = new QPushButton("Cancel");

    QObject::connect(btnok, SIGNAL(clicked()), this, SLOT(onOK()));
    QObject::connect(btncancel, SIGNAL(clicked()), this, SLOT(onCancel()));

    QHBoxLayout* lay2 = new QHBoxLayout();
    lay2->addWidget(btnok);
    lay2->addWidget(btncancel);

    QVBoxLayout* lay = new QVBoxLayout();
    lay->addWidget(mStackedWidget);
    lay->addWidget(frame);
    lay->addLayout(lay2);

    setLayout(lay);
    setWindowTitle("Select operation");
}

void OperationDialog::createOperationParametersWidgets()
{
    // This is the one and only place where to add operation parameters widgets so that they and their corresponding operations are available in the ui.

    mOperationParametersWidgets.push_back( new CameraCalibrationParametersWidget() );
    mOperationParametersWidgets.push_back( new StereoRigCalibrationParametersWidget() );
    mOperationParametersWidgets.push_back( new MonoRecordingParametersWidget() );
    mOperationParametersWidgets.push_back( new StereoRecordingParametersWidget() );
}

OperationPtr OperationDialog::getOperation()
{
    return mOperation;
}

int OperationDialog::exec()
{
    mOperation.reset();
    return QDialog::exec();
}

QWidget* OperationDialog::createHomeWidget()
{
    QVBoxLayout* lay = new QVBoxLayout();
    mBtnGroup = new QButtonGroup(this);

    for(int i=0; i<mOperationParametersWidgets.size(); i++)
    {
        QRadioButton* btn = new QRadioButton(mOperationParametersWidgets[i]->name());
        mBtnGroup->addButton(btn, i);
        lay->addWidget(btn);

        if( i == 0 )
        {
            btn->setChecked(true);
        }
    }

    QWidget* ret = new QWidget();
    ret->setLayout(lay);

    return ret;
}

OperationParametersWidget* OperationDialog::currentOperationParametersWidget()
{
    OperationParametersWidget* ret = nullptr;

    const int id = mBtnGroup->checkedId();

    if( 0 <= id && id < mOperationParametersWidgets.size() )
    {
        ret = mOperationParametersWidgets[id];
    }

    return ret;
}

void OperationDialog::onOK()
{
    OperationParametersWidget* w = currentOperationParametersWidget();
    
    if( w != nullptr )
    {
        if( mStackedWidget->currentWidget() == mPageHome )
        {
            mStackedWidget->setCurrentWidget(w);
        }
        else
        {
            mOperation = w->getOperation();
            if( mOperation )
            {
                accept();
            }
        }
    }
}

void OperationDialog::onCancel()
{
    if( mStackedWidget->currentWidget() == mPageHome )
    {
        reject();
    }
    else
    {
        mStackedWidget->setCurrentWidget(mPageHome);
    }
}

